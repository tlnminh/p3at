
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
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry);
private:
  ros::NodeHandle n;
  ros::Subscriber sub_pose;
  ros::Publisher twist_pub_;
  double linear_, angular_, l_scale_, a_scale_;
  double x, y, z, delta_x, delta_y, theta, delta_theta, vl, angular_limit;
  double x_muon, y_muon, x_first, y_first;
  bool first;
  double kp_angular, kp_linear;
  Quaterniond ori;
};
Listener::Listener():
  first(true),
  kp_angular(0.1),
  kp_linear(0.001),
  vl(0.3),
  angular_limit(0.3)
{
  sub_pose = n.subscribe("/RosAria/pose",1000, &Listener::odometryCallback, this);
  twist_pub_ = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
}
void Listener::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
// estimate with diameter = 222
  x = msg->pose.pose.position.x*1000;
  y = msg->pose.pose.position.y*1000;
  ori.x = msg->pose.pose.orientation.x;
  ori.y = msg->pose.pose.orientation.y;
  ori.z = msg->pose.pose.orientation.z;
  ori.w = msg->pose.pose.orientation.w;
  z = angle_convert(ori)*180/3.14;

  if (first == true)
  {
	  cout << "x_muon (mm): ";
  	cin >> x_muon;
  	cout << "y_muon (mm): ";
  	cin >> y_muon;
  	first = false;
  	cout << "x_muon: " << x_muon << " y_muon: " << y_muon << endl;

    x_first = x;
    y_first = y;
  }

  delta_x = x_muon - x;
  delta_y = y_muon - y;
  theta = atan2(delta_y,delta_x)*180/3.14159;
  delta_theta = theta - z;

  angular_ = kp_angular*delta_theta;

  linear_ = kp_linear*sqrt(pow(delta_y,2)+pow(delta_x,2));

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

  if ((abs(delta_x) > 10)&&(abs(delta_y) > 10))
  {
    cout << "x: " << x << " y: " <<  y << " theta: " << theta << " z: " << z << endl;
    geometry_msgs::Twist twist;
    twist.angular.z = angular_;
    twist.linear.x = linear_;  
    twist_pub_.publish(twist);
  }
  else
  {
    cout << "DA DEN MUC TIEU" << endl;
  }

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "minh_test_1");
  Listener listener;

  ros::spin();
}