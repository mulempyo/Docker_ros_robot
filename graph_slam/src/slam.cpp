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
#include <time.h>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace graph_slam{

GraphSlamNode::GraphSlamNode() : nh_(), private_nh_("~"), got_map_(false), slam_("lm_var_csparse"), transform_thread_(nullptr), scan_filter_sub_(NULL), scan_filter_(NULL) {
    map_to_odom_ = tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ));
    init();
}

GraphSlamNode::GraphSlamNode(ros::NodeHandle& nh, ros::NodeHandle& pnh):
nh_(), private_nh_("~"), got_map_(false), slam_("lm_var_csparse"), transform_thread_(nullptr), scan_filter_sub_(NULL), scan_filter_(NULL)
{
    map_to_odom_ = tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ));
    init();
}

void GraphSlamNode::publishLoop(double transform_publish_period){
    if (transform_publish_period == 0)
        return;
    ros::Rate r(1.0 / transform_publish_period);
    while (ros::ok()) {
        publishTransform();
        r.sleep();
    }
}

GraphSlamNode::~GraphSlamNode(){
    if(transform_thread_){
        transform_thread_->join();
      }
}

void GraphSlamNode::init()
{
  got_first_scan_ = false;
  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);
  // Parameters used by our GMapping wrapper
  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";

  private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);

  double tmp;
  if(!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);
  
  // Parameters used by GMapping itself
  
  if(!private_nh_.getParam("xmin", xmin_))
    xmin_ = -100.0;
  if(!private_nh_.getParam("ymin", ymin_))
    ymin_ = -100.0;
  if(!private_nh_.getParam("xmax", xmax_))
    xmax_ = 100.0;
  if(!private_nh_.getParam("ymax", ymax_))
    ymax_ = 100.0;
  if(!private_nh_.getParam("delta", delta_))
    delta_ = 0.05;
  if(!private_nh_.getParam("tf_delay", tf_delay_))
    tf_delay_ = transform_publish_period_;

}

void GraphSlamNode::startLiveSlam(){
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  map_metadata_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  map_service_ = nh_.advertiseService("dynamic_map", &GraphSlamNode::mapCallback, this);

  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_listener_, odom_frame_, 5);

  if (!scan_filter_) {
      ROS_ERROR("scan_filter_ is NULL!");
      return;
  }

  scan_filter_->registerCallback([this](const sensor_msgs::LaserScan::ConstPtr& msg) { 
      laserCallback(msg); 
  });

  transform_thread_ = new boost::thread(boost::bind(&GraphSlamNode::publishLoop, this, transform_publish_period_));
}

bool GraphSlamNode::getOdomPose(Eigen::Vector3d& map_pose, const ros::Time& t){
     
     centered_laser_pose_.stamp_ = t;
  
     tf::Stamped<tf::Transform> odom_pose;
     try
     {
       tf_listener_.transformPose(odom_frame_, centered_laser_pose_, odom_pose);
     }
    catch(tf::TransformException e)
     {
       ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
       return false;
     }
     double yaw = tf::getYaw(odom_pose.getRotation());

     map_pose = Eigen::Vector3d(odom_pose.getOrigin().x(),
                                odom_pose.getOrigin().y(),
                                yaw);
     return true;
}

bool GraphSlamNode::initMapper(const sensor_msgs::LaserScan& scan)
{
  laser_frame_ = scan.header.frame_id;

  tf::Stamped<tf::Pose> ident;
  tf::Stamped<tf::Transform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = laser_frame_;
  ident.stamp_ = scan.header.stamp;

  try
  {
    tf_listener_.transformPose(base_frame_, ident, laser_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)", e.what());
    return false;
  }

  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp, base_frame_);

  try
  {
    tf_listener_.transformPoint(laser_frame_, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN("Unable to determine orientation of laser: %s", e.what());
    return false;
  }

  if (fabs(fabs(up.z()) - 1) > 0.001)
  {
    ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f", up.z());
    return false;
  }

  laser_beam_count_= scan.ranges.size();

  double angle_center = (scan.angle_min + scan.angle_max) / 2;
  if (up.z() > 0)  
  {
    centered_laser_pose_ = tf::Stamped<tf::Pose>(
        tf::Transform(tf::createQuaternionFromRPY(0, 0, angle_center), tf::Vector3(0, 0, 0)), 
        ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upwards.");
  }
  else  
  {
    centered_laser_pose_ = tf::Stamped<tf::Pose>(
        tf::Transform(tf::createQuaternionFromRPY(M_PI, 0, -angle_center), tf::Vector3(0, 0, 0)), 
        ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upside down.");
  }

  Eigen::Vector3d initialPose;
  if(!getOdomPose(initialPose, scan.header.stamp))
  {
    ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
    initialPose = Eigen::Vector3d(0.0, 0.0, 0.0);
  }

  return true;
}

bool GraphSlamNode::addScan(const sensor_msgs::LaserScan& scan, Eigen::Vector3d& odom_pose){

  if(scan.ranges.size() != laser_beam_count_){
    return false;
  }
 
  g2o::VertexSE2* new_node = slam_.add_se2_node(odom_pose);
 
  if (new_node == nullptr) {
    return false;
  }

  if (!new_node) {
      ROS_ERROR("Failed to add new node to graph!");
      return false;
  }

  if (slam_.num_vertices() > 1) {

      int prev_id = slam_.num_vertices() - 2;

      if (prev_id < 0) {
         ROS_ERROR("Invalid previous node ID: %d, skipping edge creation.", prev_id);
        return false;
      }
      g2o::VertexSE2* prev_node = dynamic_cast<g2o::VertexSE2*>(slam_.getGraph()->vertex(prev_id));

      if (prev_node == nullptr) {
          return false;
      }

      if (!prev_node) {
          ROS_ERROR("Previous node (ID: %d) is NULL, skipping edge creation.", prev_id);
          return false;
      }
      
      Eigen::Vector3d relative_pose = slam_.compute_scan_matching(current_scan, past_scans_.back());
      slam_.add_se2_edge(prev_node, new_node, relative_pose, Eigen::Matrix3d::Identity());
      g2o::EdgeSE2* edge = slam_.add_se2_edge(prev_node, new_node, relative_pose, Eigen::Matrix3d::Identity());
      }

      past_scans_.push_back(current_scan);


      if (past_scans_.size() > 5) {  
        slam_.detect_loop_closure(slam_, past_scans_, current_scan);
      }
      
      static ros::Time last_optimization_time = ros::Time::now();
      if ((ros::Time::now() - last_optimization_time).toSec() > 0) {  
        slam_.optimize(15);
        last_optimization_time = ros::Time::now();
      }
      return true;
}

void GraphSlamNode::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
   
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;
 
  current_scan = laserScanToPointCloud(scan);
  if (current_scan->empty()) {
      return;
  }

  static ros::Time last_map_update(0,0);

  if (!got_first_scan_) {
      if (!initMapper(*scan)) {
          return;
      }
      got_first_scan_ = true;
  }

  tf::StampedTransform odom_transform;
  try {
      tf_listener_.lookupTransform(odom_frame_, scan->header.frame_id, ros::Time(0), odom_transform);
  } catch (tf::TransformException &e) {
      ROS_WARN("Failed to get odom transform: %s", e.what());
      return;
  }

  Eigen::Vector3d odom_pose(odom_transform.getOrigin().x(),
                            odom_transform.getOrigin().y(),
                            tf::getYaw(odom_transform.getRotation()));

  if(!getOdomPose(odom_pose, scan->header.stamp)){
    return;
  }
                     
  if(addScan(*scan, odom_pose)){                      
 
  Eigen::Vector3d mpose = slam_.getOptimizedPose();  
  if (std::isnan(mpose[0]) || std::isnan(mpose[1]) || std::isnan(mpose[2])) {
    ROS_ERROR("Optimized pose contains NaN values! Skipping TF update.");
    return;
  }

  tf::StampedTransform map_to_laser;
  tf_listener_.waitForTransform(map_frame_, laser_frame_, ros::Time(0), ros::Duration(0.5));
  tf_listener_.lookupTransform(map_frame_, laser_frame_, ros::Time(0), map_to_laser);

  tf::StampedTransform odom_to_laser;
  tf_listener_.waitForTransform(odom_frame_, laser_frame_, ros::Time(0), ros::Duration(0.5));
  tf_listener_.lookupTransform(odom_frame_, laser_frame_, ros::Time(0), odom_to_laser);

  map_to_odom_mutex_.lock();
  map_to_odom_ = map_to_laser * odom_to_laser.inverse();
  map_to_odom_mutex_.unlock();

  if (!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_) {
    updateMap(scan);
    last_map_update = scan->header.stamp;
  }
 }else{
    ROS_DEBUG("cannot process scan");
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

void GraphSlamNode::updateMap(const sensor_msgs::LaserScan::ConstPtr& scan) {
  boost::mutex::scoped_lock map_lock(map_mutex_);

  if (!got_map_) {
      map_.info.resolution = delta_;
      map_.header.frame_id = map_frame_;
      map_.info.width = static_cast<unsigned int>((xmax_ - xmin_) / delta_);
      map_.info.height = static_cast<unsigned int>((ymax_ - ymin_) / delta_);
      map_.info.origin.position.x = xmin_;
      map_.info.origin.position.y = ymin_;
      map_.info.origin.position.z = 0.0;
      map_.info.origin.orientation.w = 1.0;
      map_.data.resize(map_.info.width * map_.info.height);
      map_.data.assign(map_.info.width * map_.info.height, -1);
  }

  Eigen::Vector3d optimized_pose = slam_.getOptimizedPose();
  double robot_x = optimized_pose[0];
  double robot_y = optimized_pose[1];
  double robot_theta = optimized_pose[2];

  ROS_INFO("Updating map based on robot pose: x=%f, y=%f, theta=%f", robot_x, robot_y, robot_theta);

  if (!past_scans_.empty()) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr last_scan = past_scans_.back();

      for (const auto& point : last_scan->points) {
          tf::StampedTransform laser_to_map;
          try {
             tf_listener_.lookupTransform(map_frame_, laser_frame_, ros::Time(0), laser_to_map);
          } catch (tf::TransformException &ex) {
             ROS_ERROR("Failed to lookup transform: %s", ex.what());
             return;
          }

          tf::StampedTransform base_to_map;
          try {
             tf_listener_.lookupTransform(map_frame_, base_frame_, ros::Time(0), base_to_map);
          } catch (tf::TransformException &ex) {
             ROS_ERROR("Failed to lookup transform base_link to map: %s", ex.what());
            return;
          }

          tf::Vector3 transformed_point = laser_to_map * tf::Vector3(point.x, point.y, 0.0);
          tf::Vector3 transformed_robot = base_to_map * tf::Vector3(robot_x, robot_y, robot_theta);

          double worldX = transformed_point.x();
          double worldY = transformed_point.y();
          double robotX = transformed_robot.x();
          double robotY = transformed_robot.y();

          int map_x = static_cast<int>((worldX - xmin_) / delta_);
          int map_y = static_cast<int>((worldY - ymin_) / delta_);
          int robot_map_x = static_cast<int>((robotX - xmin_) / delta_);
          int robot_map_y = static_cast<int>((robotY - ymin_) / delta_);

          drawLine(robot_map_x, robot_map_y, map_x, map_y, scan);

          if (map_x >= 0 && map_x < map_.info.width && map_y >= 0 && map_y < map_.info.height) {
              map_.data[MAP_IDX(map_.info.width, map_x, map_y)] = 100;
              map_.data[MAP_IDX(map_.info.width, robot_map_x, robot_map_y)] = 0; 
          }
      }
  }

  got_map_ = true;
  map_.header.stamp = ros::Time::now();
  map_.header.frame_id = tf_listener_.resolve(map_frame_);
  map_pub_.publish(map_);
  map_metadata_pub_.publish(map_.info);
}

/* x0:robot x, y0: roboy y, x1: laserpoint x, y1: laserpoint y */
//drawLine: it use Bresenham algorithm

void GraphSlamNode::drawLine(int x0, int y0, int x1, int y1, const sensor_msgs::LaserScan::ConstPtr& scan) {
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  log_odds_map.resize(map_.info.width * map_.info.height);
  const double lmin = -2.0, lmax = 2.0; 
  const double l_occupied = 0.4;
  const double l_free = -0.4;

while (true) {
    if (x0 >= 0 && x0 < map_.info.width && y0 >= 0 && y0 < map_.info.height) {
        int idx = MAP_IDX(map_.info.width, x0, y0);
        map_.data[MAP_IDX(map_.info.width, x0, y0)] = 0;

        log_odds_map[idx] = std::max(lmin, log_odds_map[idx] + l_free);
    }

    if (x0 == x1 && y0 == y1) break;

    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 < dx) { err += dx; y0 += sy; }

    if (x1 >= 0 && x1 < map_.info.width && y1 >= 0 && y1 < map_.info.height) {
       int idx = MAP_IDX(map_.info.width, x1, y1);
       map_.data[MAP_IDX(map_.info.width, x1, y1)] = 100;
       log_odds_map[idx] = std::min(lmax, log_odds_map[idx] + l_occupied);
    }
}

for (int i = 0; i < map_.info.width * map_.info.height; ++i) {
    float odds = std::exp(log_odds_map[i]);
    float prob = odds / (1.0f + odds);

    if (prob > 0.7f)
        map_.data[i] = 100;
    else if (prob < 0.3f)
        map_.data[i] = 0;
    else
        map_.data[i] = -1;
}
}

bool GraphSlamNode::mapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res) {
  boost::mutex::scoped_lock map_lock (map_mutex_);

    if (got_map_ && map_.info.width && map_.info.height) {
        res.map = map_;
        return true;
    }
    return false;
}

void GraphSlamNode::publishTransform()
{
    map_to_odom_mutex_.lock();
    ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
    tfB_->sendTransform(tf::StampedTransform(map_to_odom_, tf_expiration, map_frame_, odom_frame_));
    map_to_odom_mutex_.unlock();
}

}
