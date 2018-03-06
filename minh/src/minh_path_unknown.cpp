#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "geometry_msgs/Twist.h"
//#include "hcr_vip/sonar_vip.h"
//#include "rosaria_client/sonar_vip.h"
#include <signal.h>
#include <termios.h>
#include "sensor_msgs/PointCloud.h"
#include "math.h"
#include "string.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include <nav_msgs/Odometry.h>
#define PI 3.14159265
using geometry_msgs::Twist;
using namespace std;
int begin = 1;
//float sonar_truoc = 0;

struct Quaterniond
{
  float x;
  float y;
  float z;
  float w;
};
struct Euler
{
  float roll;
  float pitch;
  float yaw;
};
float angle_convert(Quaterniond q)
{
  Euler angle;
  double ysqr = q.y * q.y;

  // roll (x-axis rotation)
  double t0 = +2.0 * (q.w * q.x + q.y * q.z);
  double t1 = +1.0 - 2.0 * (pow(q.x,2) + pow(q.y,2));
  angle.roll = std::atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0 * (q.w * q.y - q.z * q.x);
  t2 = ((t2 > 1.0) ? 1.0 : t2);
  t2 = ((t2 < -1.0) ? -1.0 : t2);
  angle.pitch = std::asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (q.w * q.z + q.x * q.y);
  double t4 = +1.0 - 2.0 * (pow(q.y,2) + pow(q.z,2));  
  angle.yaw = std::atan2(t3, t4);

  return angle.yaw;
}

float khoang_cach(float a, float b, float c, float d)
{
  float dis;
  dis = sqrt(pow(c-a,2)+pow(d-b,2));
  return dis;
}


class Listener
{
public:
  Listener();
  void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& pointCloud);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry);

private:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Subscriber sub_pose;
  ros::Publisher twist_pub_;
  // KHAI BAO CHO PATH PLANNING
  double linear_, angular_, l_scale_, a_scale_;
  double s7, s8, e1, e2, h, part1, vr, omega, omega_mu, delta_omega_mu, d0, k1, k2, sampling_time, angular_limit;
  double gan, xa, vua, nhe, manh, s4, s4_gan, s4_xa, s0, s5;
  Quaterniond ori;
  bool flag_pp;

  // KHAI BAO CHO WALL FOLLOWING
  double x, y, z, delta_x, delta_y, theta, delta_theta, vl;
  double x_muon, y_muon, x_first, y_first;
  bool first;
  double kp_angular, kp_linear;
  double s_ref, s;
  bool flag_wf;

  // KHAI BAO CHO PHAN CHUYEN
  float kc_chuyen;
  bool flag_chuyen;

};
Listener::Listener():
  linear_(0),
  angular_(0),
  l_scale_(3.0),
  a_scale_(1.0),
  h(29.0),
  vr(0.1),
  d0(50.0), //KHOANG CACH TUONG
  sampling_time(0.1),
  angular_limit(0.4),
  flag_wf(false),

  // khai bao cho path_finding
  first(true),
  kp_angular(0.1),
  kp_linear(1),
  vl(0.4),
  s_ref(0.0),
  flag_pp(false),

  // CHUNG
  kc_chuyen(80),
  flag_chuyen(false)
{
  sub = n.subscribe("/RosAria/sonar", 1000, &Listener::sonarCallback, this);
  sub_pose = n.subscribe("/RosAria/pose",1000, &Listener::odometryCallback, this);
  twist_pub_ = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
}
void Listener::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
// estimate with diameter = 222


  //PATH PLANNING
  x = msg->pose.pose.position.x*100;
  y = msg->pose.pose.position.y*100;
  ori.x = msg->pose.pose.orientation.x;
  ori.y = msg->pose.pose.orientation.y;
  ori.z = msg->pose.pose.orientation.z;
  ori.w = msg->pose.pose.orientation.w;
  z = angle_convert(ori)*180/3.14;

  if (first == true)
  {
    cout << "x_muon (cm): ";
    cin >> x_muon;
    cout << "y_muon (cm): ";
    cin >> y_muon;
    
    cout << "x_muon: " << x_muon << " y_muon: " << y_muon << endl;

    x_first = x;
    y_first = y;

    first = false;

    // NHAP XONG THI CHO CHAY PP TRUOC
    flag_pp = true;
  }
  delta_x = x_muon - x;
  delta_y = y_muon - y;
  theta = std::atan2(delta_y,delta_x)*180/3.14159;
  delta_theta = theta - z;

if (flag_pp == true)
{
  angular_ = kp_angular*delta_theta;

  linear_ = kp_linear*sqrt(pow(delta_y,2)+pow(delta_x,2))/100;

  if (linear_ > vl)
  {
    linear_ = vl;
  }
  else if (linear_ < -vl)
  {
    linear_ = -vl;
  }
  if (angular_ > angular_limit)
  {
    angular_ = angular_limit;
  }
  else if (angular_ < -angular_limit)
  {
    angular_ = -angular_limit;
  }
  if (abs(delta_theta) > 30)
  {
    linear_ = 0;
  }
  if ((x_muon-x_first)*(x_muon-x)+(y_muon-y_first)*(y_muon-y)<0)
  {
    linear_ = -linear_;
  }
//  cout << "x: " << x << " y: " <<  y << " theta: " << theta << " z: " << z << endl;

  geometry_msgs::Twist twist;
  twist.angular.z = angular_;
  twist.linear.x = linear_;  
  twist_pub_.publish(twist);
}
}

void Listener::sonarCallback(const sensor_msgs::PointCloud::ConstPtr& pointCloud)
{
  // KHAI BAO CHO WF
  gan = 0.8*d0;
  xa = 1.2*d0;
  nhe = 0.3*angular_limit;
  vua = 0.6*angular_limit;
  manh = angular_limit;
  s4_gan = d0 + 20;
  s4_xa = s4_gan + 30;


  bool dirty = false;
  bool chi_queo = false;
  s7 = -98.96642*(pointCloud->points[7].y)-23.1195;
  s8 = -98.96642*(pointCloud->points[8].y)-23.1195;
  s4 = 98.96642*(pointCloud->points[4].x)-23.1195;
  s5 = 98.96642*(pointCloud->points[5].x)-23.1195;
  s0 = 98.96642*(pointCloud->points[0].y)-23.1195;

  // XET FLAG
  if (flag_pp == true)
  {
    cout << "path planning   ";
    cout << " delta_x: " << delta_x << " delta_y: " << delta_y << " delta_theta: " << delta_theta << endl;
    if (s4 < kc_chuyen)
    {
      flag_pp = false;
      flag_chuyen = true;
    }
  }
  if (flag_chuyen == true)
  {
    cout << "path planning -> wall following " << endl;
    angular_ = vua;
    geometry_msgs::Twist twist;
    twist.angular.z = angular_;

    twist_pub_.publish(twist);

    if ((abs(s7 - s8) < 5)&&(s7<100)&&(s8<100))
    {
      flag_chuyen = false;
      flag_wf = true;
    }
  }
  if (flag_wf == true)
  {
    cout << "wall following   ";
    cout << " delta_x: " << delta_x << " delta_y: " << delta_y << " delta_theta: " << delta_theta << endl;
    // TRONG LUC WALL FOLLOWING (delta_theta luon nho hon 0 )
    if (delta_theta > 10)
    {
      flag_pp = true;
      flag_wf = false;

      x_first = x;
      y_first = y;
    }
  }
  // NEU DA DAT DEN DIEM x_muon, y_muon THI DUNG LAI
  if ((abs(delta_x) < 5)&&(abs(delta_y) < 5))
  {
    flag_pp = false;
    flag_wf = false;
    flag_chuyen = false;

    cout << "DA DEN MUC TIEU" << endl;
  }

  // WALL FOLLOWING
  /* s7, s8
  G G
  G D
  G X

  D G
  D D
  D X

  X G
  X D
  X X
  */
if (flag_wf == true)
{
//  if ((s4 > s4_gan) && (s5 > s4_gan))
//  {
    if ((s7 < gan) && (s8 < gan))
    {
      angular_ = vua;
      dirty = true;
    }
    else if ((s7 < gan) && (s8 > gan) && (s8 < xa))
    {
     angular_ = nhe;
     dirty = true;
    }
    else if ((s7 < gan) && (s8 > xa))
    {
      angular_ = manh;
      dirty = true;
      chi_queo = true;
    }
    else if ((s7 > gan) && (s7 < xa) && (s8 < gan))
    {
      angular_ = -nhe;
      dirty = true;
    }
    else if ((s7 > gan) && (s7 < xa) && (s8 > gan) && (s8 < xa))
    {
      angular_ = 0;
      dirty = true;
    }
    else if ((s7 > gan) && (s7 < xa) && (s8 > xa))
    {
      angular_ = nhe;
      dirty = true;
    }
    else if ((s7 > xa) && (s8 < gan))
    {
      angular_ = -manh;
      dirty = true;
      chi_queo = true;
    }
    else if ((s7 > xa) && (s8 > gan) && (s8 < xa))
    {
      angular_ = -nhe;
      dirty = true;
    }
    else if ((s7 > xa) && (s8 > xa))
    {
      angular_ = -vua;
      dirty = true;
    }
    if (chi_queo == false) 
    {
      if (s4 > s4_xa)
      {
        linear_ = vr;
      }
      else if ((s4 < s4_xa) && (s4 > s4_gan))
      {
        linear_ = 0.5*vr;
      }
    }
    else if(chi_queo == true)
    {
      linear_ = 0;
    }
    chi_queo = false;

    geometry_msgs::Twist twist;
    twist.angular.z = angular_;
    twist.linear.x = linear_;

    if(dirty ==true)
    {
      twist_pub_.publish(twist);
      dirty=false;
    }
  }
//}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "minh_test_1");
  Listener listener;
  ros::spin();
}