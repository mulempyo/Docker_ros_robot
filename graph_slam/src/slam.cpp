#include <slam_algorithm.h>
#include <slam.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace graph_slam{

GraphSlamNode::GraphSlamNode() : nh_(), private_nh_("~"), got_map_(false), slam_("lm_var") {}

GraphSlamNode::GraphSlamNode(ros::NodeHandle& nh, ros::NodeHandle& pnh):
nh_(), private_nh_("~"), got_map_(false), slam_("lm_var") {}

void GraphSlamNode::startLiveSlam(){
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    map_metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    map_service_ = nh_.advertiseService("dynamic_map", &GraphSlamNode::mapCallback, this);
    laser_sub_ = nh_.subscribe("scan", 10, &GraphSlamNode::laserCallback, this);
    
    double transform_publish_period;
    if (!private_nh_.getParam("map_update_interval", transform_publish_period)) {
       transform_publish_period = 5.0;  
    }
    map_update_interval_ = ros::Duration(transform_publish_period);
}

GraphSlamNode::~GraphSlamNode() {}

void GraphSlamNode::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_scan = laserScanToPointCloud(scan);
    
    tf::StampedTransform odom_transform;
    try {
        tf_listener_.lookupTransform("odom", scan->header.frame_id, scan->header.stamp, odom_transform);
    } catch (tf::TransformException &e) {
        ROS_WARN("Failed to get odom transform: %s", e.what());
        return;
    }

    Eigen::Vector3d odom_pose(odom_transform.getOrigin().x(),
                              odom_transform.getOrigin().y(),
                              tf::getYaw(odom_transform.getRotation()));

    g2o::VertexSE2* new_node = slam_.add_se2_node(odom_pose);

    if (slam_.num_vertices() > 1) {
        g2o::VertexSE2* prev_node = dynamic_cast<g2o::VertexSE2*>(slam_.getGraph()->vertex(slam_.num_vertices() - 2));

        Eigen::Vector3d relative_pose = slam_.compute_scan_matching(current_scan, past_scans_.back()); 
        slam_.add_se2_edge(prev_node, new_node, relative_pose, Eigen::Matrix3d::Identity());
    }

    past_scans_.push_back(current_scan);

    if (past_scans_.size() > 5) {  
        slam_.detect_loop_closure(slam_, past_scans_, current_scan);
    }

    static ros::Time last_optimization_time = ros::Time::now();
    if ((ros::Time::now() - last_optimization_time).toSec() > 5.0) {  
        slam_.optimize(10);
        last_optimization_time = ros::Time::now();
    }

    static ros::Time last_map_update(0, 0);
    if (!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_) {
        updateMap();
        last_map_update = scan->header.stamp;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GraphSlamNode::laserScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    double angle = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        if (std::isfinite(scan->ranges[i])) {
            pcl::PointXYZ point;
            point.x = scan->ranges[i] * cos(angle);
            point.y = scan->ranges[i] * sin(angle);
            point.z = 0.0;  

            cloud->push_back(point);
        }
        angle += scan->angle_increment;
    }
    return cloud;
}


void GraphSlamNode::updateMap() {
    
    int size_x = 200, size_y = 200;
    double resolution = 0.05;
    map_.info.resolution = resolution;
    map_.info.width = size_x;
    map_.info.height = size_y;
    map_.info.origin.position.x = -5.0;
    map_.info.origin.position.y = -5.0;
    map_.data.resize(size_x * size_y, -1);

    for (int i = 0; i < size_x; i++) {
        for (int j = 0; j < size_y; j++) {
            if ((i + j) % 3 == 0) { 
                map_.data[MAP_IDX(size_x, i, j)] = 100; 
            } else {
                map_.data[MAP_IDX(size_x, i, j)] = 0; 
            }
        }
    }

    got_map_ = true;
    map_.header.stamp = ros::Time::now();
    map_.header.frame_id = "map";
    map_pub_.publish(map_);
    map_metadata_pub_.publish(map_.info);
}

bool GraphSlamNode::mapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res) {
    if (got_map_) {
        res.map = map_;
        return true;
    }
    return false;
}

void GraphSlamNode::publishTransform() {
    ros::Rate rate(10.0);
    while (ros::ok()) {
        tf_broadcaster_.sendTransform(tf::StampedTransform(map_to_odom_, ros::Time::now(), "map", "odom"));
        rate.sleep();
    }
 }
}


