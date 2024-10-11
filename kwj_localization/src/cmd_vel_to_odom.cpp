/*
Publish: /odom
Subscribe: /cmd_vel

this code is Odometry code
it calculate odometry using /cmd_vel
this code publish /odom topic after apply covariance
*/





#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <geometry_msgs/Twist.h>

// Create odometry data publishers
ros::Publisher odom_data_pub_quat;

nav_msgs::Odometry odom;
const double PI = 3.141592;

double dist;
double dth;
double dx;
double dy;
double x=0;
double y=0;
double th=0;
double dt;
double vx=0;
double vth=0;
double linear_velocity;
double angular_velocity;
double imu_yaw_rate = 0.0;
double WHEEL_BASE = 0.212;

 
ros::Time current_time;
ros::Time last_time;

geometry_msgs::Twist cmd_vel_;

using namespace std;

void cmdCallback(const geometry_msgs::Twist cmd_vel){
    cmd_vel_ = cmd_vel;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    imu_yaw_rate = msg->angular_velocity.z;
}
 
// Update odometry information
void update_odom() { 
   current_time = ros::Time::now();
   dt =(current_time-last_time).toSec();
   
   double left_velocity;
   double right_velocity;

   left_velocity = cmd_vel_.linear.x - (cmd_vel_.angular.z*WHEEL_BASE/2.0); 
   right_velocity = cmd_vel_.linear.x + (cmd_vel_.angular.z*WHEEL_BASE/2.0);
 
   vx = (right_velocity + left_velocity)/2; 
   vth = (right_velocity - left_velocity)/WHEEL_BASE;

   dist = vx * dt;

   dth = imu_yaw_rate * dt;
   dx = dist*cos(th);
   dy = dist*sin(th);

   x += dx;
   y += dy;
   th += dth;
   th = fmod(th + 2 * PI, 2 * PI);

   geometry_msgs::Quaternion odom_quat =tf::createQuaternionMsgFromYaw(th);

   odom.header.stamp = current_time;
   odom.header.frame_id = "odom";
   odom.child_frame_id = "base_footprint";
 
   odom.pose.pose.position.x = x; 
   odom.pose.pose.position.y = y;
   odom.pose.pose.orientation = odom_quat;

   if (dt > 0) {
     odom.twist.twist.linear.x = vx;
     odom.twist.twist.linear.y = 0;
     odom.twist.twist.angular.z = vth;
   } else {
     odom.twist.twist.linear.x = 0;
     odom.twist.twist.linear.y = 0;
     odom.twist.twist.angular.z = 0;
   }
 
   last_time = current_time;
  

   for(int i = 0; i<36; i++) {
     if(i == 0 || i == 7 || i == 14) {
       odom.pose.covariance[i] = 0.01;
      }
      else if (i == 21 || i == 28 || i== 35) {
        odom.pose.covariance[i] = 0.1;
      }
      else {
        odom.pose.covariance[i] = 0;
      }
   } 
   
   odom_data_pub_quat.publish(odom);
}
 
int main(int argc, char **argv) {
   
  // Launch ROS and create a node
  ros::init(argc, argv, "cmd_vel_to_odom");
  ros::NodeHandle node;

  // Subscribe to ROS topics
  ros::Subscriber subForImu = node.subscribe("/imu/data", 100, imuCallback);
  ros::Subscriber subForRightCounts = node.subscribe("cmd_vel", 10, cmdCallback);
  //ros::Subscriber sub = node.subscribe("initialpose", 10, initialPoseCallback);
  
  // Publisher of full odom message where orientation is quaternion
  odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("odom", 100);
  ros::Rate r(30);
   while(ros::ok()){
     update_odom();
     ros::spinOnce();
     r.sleep();
   }
   return 0;
}
