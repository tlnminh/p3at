#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

//  for rosaria //
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

struct Quaterniond
{
  float x;
  float y;
  float z;
  float w;
};

struct Position
{
  float x;
  float y;
  float z;
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

Quaterniond ori;
Position pos;
double yaw_converted;

void p3at_Pose(const nav_msgs::Odometry::ConstPtr& msg)
{
    pos.x = msg->pose.pose.position.x;
    pos.y = msg->pose.pose.position.y;
    pos.z = msg->pose.pose.position.z;

    ori.x = msg->pose.pose.orientation.x;
    ori.y = msg->pose.pose.orientation.y;
    ori.z = msg->pose.pose.orientation.z;
    ori.w = msg->pose.pose.orientation.w;

    yaw_converted = angle_convert(ori)*180/3.14;
//    ROS_INFO("Position-> x: [%f]",pos.x);
}
    
int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;

    // publish joint state
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    // subscribe for p3at pose
    ros::Subscriber sub = n.subscribe("/RosAria/pose",1000, p3at_Pose); 

    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(4);
        joint_state.position.resize(4);
        joint_state.name[0] ="front_right_wheel_joint";
        joint_state.position[0] = swivel;
        joint_state.name[1] ="back_right_wheel_joint";
        joint_state.position[1] = swivel;
        joint_state.name[2] ="front_left_wheel_joint";
        joint_state.position[2] = swivel;
        joint_state.name[3] ="back_left_wheel_joint";
        joint_state.position[3] = swivel;

        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = pos.x;
        odom_trans.transform.translation.y = pos.y;
        odom_trans.transform.translation.z = pos.z;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(yaw_converted);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // Create new robot state
        swivel += degree;

        // This will adjust as needed per iteration
        loop_rate.sleep();

//        ros::spin();//run it until you press ctrl+c
    }

    ros::spin();//run it until you press ctrl+c
    return 0;
}