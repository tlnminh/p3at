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
};
Listener::Listener():
  linear_(0),
  angular_(0),
  l_scale_(3.0),
  a_scale_(1.0),
  h(29.0),
  vr(0.1),
  d0(40.0),
  k1(3.5),
  k2(615.0),
  sampling_time(0.1),
  angular_limit(0.3)
{
  sub = n.subscribe("/RosAria/sonar", 1000, &Listener::sonarCallback, this);
  sub_pose = n.subscribe("/RosAria/pose",1000, &Listener::odometryCallback, this);
  twist_pub_ = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
}
void Listener::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
// estimate with diameter = 222
  cout << "Position: x(m): " <<  int(msg->pose.pose.position.x*100) 
       << " y: " << int(msg->pose.pose.position.y*100)
       << " Orientation z: " <<  asin(msg->pose.pose.orientation.z)*2*180/3.14
       << endl;
//  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
//  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
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


  gan = 0.85*d0;
  xa = 1.15*d0;
  nhe = 0.3*angular_limit;
  vua = 0.6*angular_limit;
  manh = angular_limit;
  s4_gan = 80;
  s4_xa = 100;
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

//  cout << "angular_: " << angular_ << " linear_: " << linear_ << endl;
/*
  e2=atan((d1-d2)/h);
  e1=d0-d1*cos(e2);

  part1=vr-(e1+d0)*omega_mu/cos(e2);
  omega=k2*part1*e1-k1*sin(e2)+omega_mu;
  delta_omega_mu=e1*(e1+d0)*tan(e2)-sin(e2)/k2;
  delta_omega_mu=delta_omega_mu*sampling_time;
  omega_mu +=delta_omega_mu;

  angular_ = omega*a_scale_;
  linear_ = vr*l_scale_;

  if (angular_ >= angular_limit)
  {
    angular_ = angular_limit;
  }
  if (angular_ <= -angular_limit)
  {
    angular_ = -angular_limit;
  }

  if (e2*180/3.14 > 10)
  {
    linear_ = 0;
  }
*/

/*
  sonar_truoc_x = pointCloud->points[3].x;
  sonar_truoc_y = pointCloud->points[3].y;

  sonar_sau_x = -pointCloud->points[11].x;
  sonar_sau_y = -pointCloud->points[11].y;

  cout << "Truoc_x " << sonar_truoc_x <<  " Truoc_y " << sonar_truoc_y << " Sau_x: " 
  << sonar_sau_x << " Sau_y: " << sonar_sau_y << endl;

  if (sonar_truoc_x < 1.0){
//    ROS_DEBUG("Lui");
    linear_ = -1;
    angular_ = 0;
    dirty = true;
  }
  if (sonar_sau_x < 1.0){
//    ROS_DEBUG("Lui");
    linear_ = 1;
    angular_ = 0;
    dirty = true;
  }
*/

  geometry_msgs::Twist twist;
  twist.angular.z = angular_;
  twist.linear.x = linear_;

  if(dirty ==true)
  {
    twist_pub_.publish(twist);
    dirty=false;
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "minh_test_1");
  Listener listener;
//  ros::Subscriber sub = n.subscribe("/RosAria/sonar", 1000, &Listener::sonarCallback, &listener);
//  ros::Publisher twist_pub_ = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);


  
  
  //cout << sonar_truoc <<endl;
/*
  if (sonar_truoc < 1.0){
    ROS_DEBUG("Lui");
    linear_ = -1;
    angular_ = 0;
    dirty = true;
  }

    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_.publish(twist);
      dirty=false;
    }
*/
  ros::spin();
}