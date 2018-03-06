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



class Listener
{
public:
  Listener();
  void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& pointCloud);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry);

//  void move(const geometry_msgs::Twist::ConstPtr& Twist);
private:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Subscriber sub_pose;
  ros::Publisher twist_pub_;
  double linear_, angular_, l_scale_, a_scale_;
  double s7, s8, e1, e2, h, part1, vr, omega, omega_mu, delta_omega_mu, d0, k1, k2, sampling_time, angular_limit;
  double gan, xa, vua, nhe, manh, s4, s4_gan, s4_xa, s0, s5;

  // khai bao cho path_finding
  double x, y, z, delta_x, delta_y, theta, delta_theta, vl;
  double x_muon, y_muon;
  bool first;
  double kp_angular, kp_linear;
  double s_ref, s;


};
Listener::Listener():
  linear_(0),
  angular_(0),
  l_scale_(3.0),
  a_scale_(1.0),
  h(29.0),
  vr(0.2),
  d0(30.0),
  k1(3.5),
  k2(615.0),
  sampling_time(0.1),
  angular_limit(0.3),

  // khai bao cho path_finding
  first(true),
  kp_angular(0.1),
  kp_linear(0.001),
  vl(0.3),
  s_ref(0.0)
{
  sub = n.subscribe("/RosAria/sonar", 1000, &Listener::sonarCallback, this);
  sub_pose = n.subscribe("/RosAria/pose",1000, &Listener::odometryCallback, this);
  twist_pub_ = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
}
void Listener::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
// estimate with diameter = 222

  x = msg->pose.pose.position.x*1000;
  y = msg->pose.pose.position.y*1000;
  z = asin(msg->pose.pose.orientation.z)*2*180/3.14159;

  if (first == true)
  {
  cout << "x_muon (mm): ";
    cin >> x_muon;
    cout << "y_muon (mm): ";
    cin >> y_muon;
    first = false;
    cout << "x_muon: " << x_muon << " y_muon: " << y_muon << endl;
  }

  delta_x = x_muon - x;
  delta_y = y_muon - y;
  theta = atan(delta_y/delta_x)*180/3.14159;
  delta_theta = theta - z;

if (s_ref > 80)
{
  angular_ = kp_angular*delta_theta;
  linear_ = kp_linear*sqrt(pow(delta_y,2)+pow(delta_x,2));

  if (linear_ > vl)
  {
    linear_ = vl;
  }
  if ((delta_x < 0) || (delta_y < 0))
  {
    linear_ = -linear_;
  }

  cout << "x: " << x << " y: " <<  y << " angular_: " << angular_ << " linear_: " << linear_ 
       << " delta_theta: " << delta_theta << endl;

  geometry_msgs::Twist twist;
  twist.angular.z = angular_;
  twist.linear.x = linear_;  
  twist_pub_.publish(twist);
}

}
void Listener::sonarCallback(const sensor_msgs::PointCloud::ConstPtr& pointCloud)
{
//  cout << "So cam bien: " << pointCloud->points.size() << endl;
//  cout << pointCloud->points[3].x << endl;
  bool dirty = false;
  bool ko_queo = false;
  s7 = -98.96642*(pointCloud->points[7].y)-23.1195;
  s8 = -98.96642*(pointCloud->points[8].y)-23.1195;
  s4 = 98.96642*(pointCloud->points[4].x)-23.1195;
  s5 = 98.96642*(pointCloud->points[5].x)-23.1195;
  s0 = 98.96642*(pointCloud->points[0].y)-23.1195;

  if ((abs(delta_theta)  > 0) && (abs(delta_theta)  < 20))
  {
    s_ref = 98.96642*(sqrt(pow(pointCloud->points[4].x,2)+pow(pointCloud->points[4].y,2)))-23.1195;
    s = 4;
  }
  else if ((abs(delta_theta)  > 20) && (abs(delta_theta)  < 40))
  {
    s_ref = 98.96642*(sqrt(pow(pointCloud->points[5].x,2)+pow(pointCloud->points[5].y,2)))-23.1195;
    s = 5;
  }
  else if ((abs(delta_theta)  > 40) && (abs(delta_theta)  < 70))
  {
    s_ref = 98.96642*(sqrt(pow(pointCloud->points[6].x,2)+pow(pointCloud->points[6].y,2)))-23.1195;
    s = 6;
  }
  else if ((abs(delta_theta)  > 70) && (abs(delta_theta)  < 90))
  {
    s_ref = 98.96642*(sqrt(pow(pointCloud->points[7].x,2)+pow(pointCloud->points[7].y,2)))-23.1195;
    s = 7;
  }
  else if ((abs(delta_theta)  > 90) && (abs(delta_theta)  < 110))
  {
    s_ref = 98.96642*(sqrt(pow(pointCloud->points[8].x,2)+pow(pointCloud->points[8].y,2)))-23.1195;
    s = 8;
  }
  else if ((abs(delta_theta)  > 110) && (abs(delta_theta)  < 140))
  {
    s_ref = 98.96642*(sqrt(pow(pointCloud->points[9].x,2)+pow(pointCloud->points[9].y,2)))-23.1195;
    s = 9;
  }
  else if ((abs(delta_theta)  > 140) && (abs(delta_theta)  < 160))
  {
    s_ref = 98.96642*(sqrt(pow(pointCloud->points[10].x,2)+pow(pointCloud->points[10].y,2)))-23.1195;
    s = 10;
  }
  else if ((abs(delta_theta)  > 160) && (abs(delta_theta)  < 180))
  {
    s_ref = 98.96642*(sqrt(pow(pointCloud->points[11].x,2)+pow(pointCloud->points[11].y,2)))-23.1195;
    s = 11;
  }
  
  gan = 0.85*d0;
  xa = 1.15*d0;
  nhe = 0.3*angular_limit;
  vua = 0.6*angular_limit;
  manh = angular_limit;
  s4_gan = 50;
  s4_xa = 80;
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
if (s_ref <80) { 

  cout << "x: " << x << " y: " << y << "delta_theta: " << delta_theta << " -> sonar: " << s << endl;

if ((s4 > s4_gan) && (s5 > s4_gan))
{
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
    ko_queo = true;
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
    ko_queo = true;
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
  if (ko_queo == false) 
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
  else if(ko_queo == true)
  {
    linear_ = 0;
  }
  ko_queo = false;
}
else
{
      linear_ = 0;
      if (s7 < 100)
      {
        angular_ = angular_limit;
        dirty = true;
      }
      if (s0 < 50)
      {
        angular_ = -angular_limit;
        dirty = true;
      }
}

  geometry_msgs::Twist twist;
  twist.angular.z = angular_;
  twist.linear.x = linear_;

  if(dirty ==true)
  {
    twist_pub_.publish(twist);
    dirty=false;
  }
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "minh_test_1");
  Listener listener;
  ros::spin();
}