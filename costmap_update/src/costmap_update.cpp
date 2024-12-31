#include <dwa_planner_ros/dwa_planner_ros.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <array>
#include <vector>

ros::Publisher marker_array_pub;
std::vector<std::array<float, 7>> safes = {
        {-0.063431f, -0.031137f, 0.0f, 0.0f, 0.0f, 0.19328f, 0.999903f},
        {4.273204f, 0.379562f, 0.0f, 0.0f, 0.0f, -0.998399f, 0.056565f},
        {0.758307f, -0.584536f, 0.0f, 0.0f, 0.0f, -0.065801f, 0.997833f},
        {1.517976f, -0.700481f, 0.0f, 0.0f, 0.0f, 0.077507f, 0.996992f},
        {2.307844f, -0.628027f, 0.0f, 0.0f, 0.0f, 0.046726f, 0.998908f},
        {3.243371f, -0.544172f, 0.0f, 0.0f, 0.0f, 0.479464f, 0.877562f},
        {2.608130f, 0.125702f, 0.0f, 0.0f, 0.0f, -0.999792f, 0.020394f},
        {3.987488f, 0.935925f, 0.0f, 0.0f, 0.0f, -0.998293f, 0.058406f},
        {2.584035f, 1.041702f, 0.0f, 0.0f, 0.0f, 0.999876f, 0.015761f},
        {1.647480f, 1.120834f, 0.0f, 0.0f, 0.0f, 0.999695f, 0.024705f},
        {0.597729f, 0.904205f, 0.0f, 0.0f, 0.0f, -0.951543f, 0.307515f}
        };

void markPublish(){
    visualization_msgs::MarkerArray marker_array;

    for(size_t i = 0; i < safes.size(); ++i){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map"; 
        marker.header.stamp = ros::Time::now();
        marker.ns = "array_namespace"; 
        marker.id = i; 
        marker.type = visualization_msgs::Marker::ARROW; 
        marker.action = visualization_msgs::Marker::ADD; 

        marker.pose.position.x = safes[i][0];
        marker.pose.position.y = safes[i][1];
        marker.pose.position.z = safes[i][2];
        marker.pose.orientation.x = safes[i][3];
        marker.pose.orientation.y = safes[i][4];
        marker.pose.orientation.z = safes[i][5];
        marker.pose.orientation.w = safes[i][6];

        tf2::Quaternion q_marker;
        tf2::fromMsg(marker.pose.orientation, q_marker);

        double additional_yaw = M_PI / 4; 
        tf2::Quaternion q_additional;
        q_additional.setRPY(0, 0, additional_yaw);

        tf2::Quaternion q_new = q_marker * q_additional;
        q_new.normalize();

        marker.pose.orientation = tf2::toMsg(q_new);

        marker.scale.x = 0.1; 
        marker.scale.y = 0.03; 
        marker.scale.z = 0.03; 

        marker.color.r = 0.0f; 
        marker.color.g = 0.0f; 
        marker.color.b = 1.0f; 
        marker.color.a = 1.0f; 

        marker.lifetime = ros::Duration();

        marker_array.markers.push_back(marker);
    }

    marker_array_pub.publish(marker_array);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "update_map");

    ros::NodeHandle nh;

    dwa_planner_ros::DWAPlannerROS dwa_;
    
    dwa_.sub_ = nh.subscribe("scan", 10, &dwa_planner_ros::DWAPlannerROS::laserCallback, &dwa_);
    dwa_.amcl_sub_ = nh.subscribe("/safe", 10, &dwa_planner_ros::DWAPlannerROS::safeMode, &dwa_);

    marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    ros::Timer timer = nh.createTimer(ros::Duration(1.0), [&](const ros::TimerEvent&) {
        markPublish();
    });
   
    ros::spin();

    return 0;
}
