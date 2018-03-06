
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

struct toa_do
{
  float x;
  float y;
  float theta;
};
float khoang_cach (struct toa_do a, struct toa_do b)
{
  float distance = sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
  return distance;
}
class Listener
{
public:
  Listener();
//  void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& pointCloud);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry);
//  void move(const geometry_msgs::Twist::ConstPtr& Twist);
private:
  ros::NodeHandle n;
//  ros::Subscriber sub;
  ros::Subscriber sub_pose;
  ros::Publisher twist_pub_;
  double linear_, angular_, l_scale_, a_scale_;
  double x, y, z, delta_x, delta_y, goc, delta_theta, vl, x_first, y_first;
  double x_muon, y_muon;
  bool first;
  double kp_angular, kp_linear;
  toa_do robot;
  toa_do dich;
  toa_do dau;
};
Listener::Listener():
  first(true),
  kp_angular(0.01),
  kp_linear(0.001),
  vl(0.3)
{
//  sub = n.subscribe("/RosAria/sonar", 1000, &Listener::sonarCallback, this);
  sub_pose = n.subscribe("/RosAria/pose",1000, &Listener::odometryCallback, this);
  twist_pub_ = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
}
void Listener::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
// estimate with diameter = 222
/*
  cout << "Position: x(m): " <<  int(msg->pose.pose.position.x*100) 
       << " y: " << int(msg->pose.pose.position.y*100)
       << " Orientation z: " <<  asin(msg->pose.pose.orientation.z)*2*180/3.14
       << endl;
*/
  robot.x = msg->pose.pose.position.x*1000;
  robot.y = msg->pose.pose.position.y*1000;
  robot.theta = asin(msg->pose.pose.orientation.z)*2*180/3.14159;

  if (first == true)
  {
	  cout << "x_muon (mm): ";
  	cin >> dich.x;
  	cout << "y_muon (mm): ";
  	cin >> dich.y;
  	first = false;
  	cout << "x_muon: " << dich.x << " y_muon: " << dich.y << endl;

    dau.x = robot.x;
    dau.y = robot.y;
  }

  delta_x = dich.x - robot.x;
  delta_y = dich.y - robot.y;
  goc = atan(delta_y/delta_x)*180/3.14159;
  delta_theta = robot.theta - goc;

  angular_ = kp_angular*delta_theta;

  linear_ = kp_linear*sqrt(pow(delta_y,2)+pow(delta_x,2));

  if (linear_ > vl)
  {
  	linear_ = vl;
  }
 
  if (khoang_cach(robot,dau)>khoang_cach(dich,dau))
  {
  	linear_ = -linear_;
  }

  if (abs(delta_theta) > 45)
  {
    linear_ = 0;
  }

  cout << "robot.x: " << robot.x << " robot.y: " <<  robot.y << " robot.theta: " << robot.theta << " goc: " << goc << " delta_theta: " << delta_theta << endl;
  geometry_msgs::Twist twist;
  twist.angular.z = angular_;
  twist.linear.x = linear_;  
  twist_pub_.publish(twist);
}
/*
void Listener::sonarCallback(const sensor_msgs::PointCloud::ConstPtr& pointCloud)
{
//  cout << "So cam bien: " << pointCloud->points.size() << endl;
//  cout << pointCloud->points[3].x << endl;
  bool dirty = false;
  geometry_msgs::Twist twist;
  twist.angular.z = angular_;
  twist.linear.x = linear_;

  if(dirty ==true)
  {
    twist_pub_.publish(twist);
    dirty=false;
  }
}
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "minh_test_1");
  Listener listener;

  ros::spin();
}