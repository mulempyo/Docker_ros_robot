#ifndef GRAPH_SLAM_2D
#define GRAPH_SLAM_2D

#include <slam_algorithm.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>

namespace graph_slam{

class GraphSlamNode {
public:
    GraphSlamNode();
    GraphSlamNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~GraphSlamNode();

    void startLiveSlam();
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool mapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);
    void publishTransform();
    void updateMap();

private:
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher map_pub_, map_metadata_pub_;
    ros::ServiceServer map_service_;
    ros::Subscriber laser_sub_;
    
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
    tf::StampedTransform map_to_odom_;

    graph_slam::GraphSLAM slam_;

    nav_msgs::OccupancyGrid map_;
    bool got_map_;
    ros::Duration map_update_interval_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> past_scans_;
};

}
#endif


