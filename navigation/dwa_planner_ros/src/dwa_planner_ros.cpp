// dwa_planner_ros.cpp
#include <chrono>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "dwa_planner_ros/dwa_planner_ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <memory>
#include <std_msgs/Float64.h>
#include <limits>
#include <torch/torch.h>
#include <astar_planner/astar.h>
#include <Eigen/Core>
#include <Eigen/Dense>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_planner_ros::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace dwa_planner_ros {

DWAPlannerROS::DWAPlannerROS()
  : initialized_(false), size_x_(0), size_y_(0), goal_reached_(false), rotate(true), tf_buffer_(), tf_listener_(tf_buffer_), fuzzy_nn(std::make_shared<FuzzyNN>())
{
    ros::NodeHandle nh;
    nh_ = nh;
    laser_sub_ = nh.subscribe("scan", 1, &DWAPlannerROS::scanCallback, this);
    person_sub_ = nh.subscribe("person_probability", 10, &DWAPlannerROS::personDetect, this);
    amcl_sub_ = nh.subscribe("/safe", 10, &DWAPlannerROS::safeMode, this);
    odom_sub_ = nh.subscribe("/odometry/filtered", 10, &DWAPlannerROS::odomSub, this);
    goal_sub_ = nh.subscribe("/goal", 1, &DWAPlannerROS::goalSub, this);
    train_timer_ = nh.createTimer(ros::Duration(0.5), &DWAPlannerROS::trainCallback, this);
    global_timer_ = nh.createTimer(ros::Duration(0.5), &DWAPlannerROS::globalReplanning, this);
}

DWAPlannerROS::~DWAPlannerROS()
{
    freeMemory();
    if (planner_)
        delete planner_;
    if (costmap_model_)
        delete costmap_model_;  
}

void DWAPlannerROS::freeMemory()
{
    if (charmap_)
    {
        for (int i = 0; i < size_x_; i++)
        {
            delete[] charmap_[i];
            charmap_[i] = nullptr;
        }
    }
}

void DWAPlannerROS::allocateMemory()
{
    if(charmap_ != nullptr){
        freeMemory();
    }

    charmap_ = new unsigned char*[size_x_];
    for (int i = 0; i < size_x_; i++)
        charmap_[i] = new unsigned char[size_y_];
}

void DWAPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!initialized_)
    { 
        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("odom_topic", odom_topic_, std::string("/odometry/filtered"));
        private_nh.param("map_frame", map_frame_, std::string("map"));
        private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.2);
        private_nh.param("transform_tolerance", transform_tolerance_, 0.5);
        private_nh.param("max_vel_x", max_vel_x_, 0.55);
        private_nh.param("min_vel_x", min_vel_x_, 0.0);
        private_nh.param("max_vel_theta", max_vel_theta_, 2.5);
        private_nh.param("min_vel_theta", min_vel_theta_, -2.5);
        private_nh.param("acc_lim_x", acc_lim_x_, 0.25);
        private_nh.param("acc_lim_theta", acc_lim_theta_, 1.2);
        private_nh.param("control_period", control_period_, 0.2);
        private_nh.param("repulsive_factor", repulsive_factor_, 1.5);
        private_nh.param("min_potential_distance", min_potential_distance_, 0.5);

        tf_ = tf;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();  
        size_x_ = costmap_->getSizeInCellsX();
        size_y_ = costmap_->getSizeInCellsY();
        size_x = size_x_;
        size_y = size_y_;
        resolution = costmap_->getResolution();
        origin_x = costmap_->getOriginX();
        origin_y = costmap_->getOriginY();

        costmap_model_ = new base_local_planner::CostmapModel(*costmap_);

        global_frame_ = costmap_ros_->getGlobalFrameID();
        robot_base_frame_ = costmap_ros_->getBaseFrameID();

        footprint_spec_ = costmap_ros_->getRobotFootprint();
        costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_);

        odom_helper_.setOdomTopic(odom_topic_);

        planner_ = new DWAPlanner(costmap_model_, footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_,
                                  private_nh);
        global_astar_pub_ = private_nh.advertise<nav_msgs::Path>("dwa_astar_plan", 1);
        global_plan_pub_ = private_nh.advertise<nav_msgs::Path>("dwa_global_plan", 1);
        //safe_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("/safe_mode", 1);
        planner_util_.initialize(tf_, costmap_, global_frame_);
        astar.initialize("dwa_planner_ros",costmap_ros_);

        allocateMemory();

        safes = {
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
        first = true;
        initialized_ = true;
        
        ROS_DEBUG("dwa_local_planner plugin initialized.");
    }
    else
    {
        ROS_WARN("dwa_local_planner has already been initialized, doing nothing.");
    }
}

void DWAPlannerROS::odomSub(const nav_msgs::Odometry odomMsg){
    _odom = odomMsg;
}

void DWAPlannerROS::goalSub(geometry_msgs::PoseStamped goal){
    try {
        goal_ = tf_buffer_.transform(goal, global_frame_, ros::Duration(1.0));
        goal_transformed_ = true;
    } catch(tf2::TransformException &ex) {
        ROS_WARN("Goal transform failed: %s", ex.what());
        goal_transformed_ = false;
    }
}

void DWAPlannerROS::scanCallback(const sensor_msgs::LaserScan& scan)
  {
    double threshold = 0.5;
    double angle = scan.angle_min;
    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
      double range = scan.ranges[i];
      if (std::isfinite(range) && range >= scan.range_min)
      {
        if (range < threshold)
        {
            geometry_msgs::PoseStamped obstacle,obstacle_detect;
            obstacle.header.frame_id = "laser_link";
            obstacle.header.stamp = ros::Time::now();
            obstacle.pose.position.x = range * std::cos(angle);
            obstacle.pose.position.y = range * std::sin(angle);
            obstacle.pose.position.z = 0.0;

            obstacle_detect = tf_buffer_.transform(obstacle, global_frame_, ros::Duration(1.0));
            double del_x = obstacle_detect.pose.position.x - robot_pose_x;
            double del_y = obstacle_detect.pose.position.y - robot_pose_y;

            distance_ = std::hypot(del_x, del_y);
             
          break;
        }
      }
      angle += scan.angle_increment;
    }
}

void DWAPlannerROS::safeMode(std_msgs::Float64 safe){

  ROS_WARN("safeMode function in dwa");
  if(safe.data == -1){
    safe_mode = false;
  }

}

void DWAPlannerROS::personDetect(const std_msgs::Float64::ConstPtr& person){
  if(person->data == 1.0){
    person_detect = true;
  }else{
    person_detect = false;
  }
}

void DWAPlannerROS::trainCallback(const ros::TimerEvent& event)  
{
    trainFuzzyNN();
}


void DWAPlannerROS::globalReplanning(const ros::TimerEvent& event){
    if(!first){
    std::vector<geometry_msgs::PoseStamped> plan;
    plan.clear();

    geometry_msgs::PoseStamped goal,start,current;
    costmap_ros_->getRobotPose(current);
    start = current;
    if(goal_transformed_){
        goal = goal_;
    }
    
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
        ROS_WARN("The start is out of the map bounds.");
    }

    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
        ROS_WARN("The goal is out of the map bounds.");
    }

    path = astar.aStarSearch(start_x, start_y, goal_x, goal_y);

    if(!path.empty()){
        path.erase(path.begin());
      }
    
      std::vector<double> x_vals, y_vals;
      for (unsigned int index : path) {
          unsigned int mx = index % size_x_;  
          unsigned int my = index / size_x_;
          double wx, wy;
          wx = costmap_->getOriginX() + mx * costmap_->getResolution();
          wy = costmap_->getOriginY() + my * costmap_->getResolution();
          x_vals.push_back(wx);
          y_vals.push_back(wy);
      }

      Eigen::VectorXd X = Eigen::Map<Eigen::VectorXd>(x_vals.data(), x_vals.size());
      Eigen::VectorXd Y = Eigen::Map<Eigen::VectorXd>(y_vals.data(), y_vals.size());

      int poly_order = 3;
      Eigen::VectorXd coeffs = astar.polyfit(X, Y, poly_order);

      double ds = 0.1;

      std::vector<geometry_msgs::PoseStamped> smooth_plan;
      geometry_msgs::PoseStamped first_pose;
      first_pose.header.frame_id = global_frame_; 
      first_pose.header.stamp = ros::Time::now();
      first_pose.pose.position.x = x_vals.front();
      first_pose.pose.position.y = y_vals.front();
      first_pose.pose.position.z = 0;
      smooth_plan.push_back(first_pose);

      double x_start = x_vals.front();
      double x_end = x_vals.back();
      double prev_x = x_start, prev_y = y_vals.front();


      for (double x = x_start; x < x_end; x += ds) {
         double y = astar.polyval(coeffs, x);
         geometry_msgs::PoseStamped pose;
         pose.header.frame_id = global_frame_;
         pose.header.stamp = ros::Time::now(); 
         pose.pose.position.x = x;
         pose.pose.position.y = y;
         pose.pose.position.z = 0;

          double dx = x - prev_x;
          double dy = y - prev_y;
          double theta = std::atan2(dy, dx);
          prev_x = x;
          prev_y = y;
  
          pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(theta/2), cos(theta/2)));
          smooth_plan.push_back(pose);
       }

       geometry_msgs::PoseStamped last_pose;
       last_pose.header.frame_id = global_frame_;
       last_pose.header.stamp = ros::Time::now();
       last_pose.pose.position.x = x_vals.back();
       last_pose.pose.position.y = y_vals.back();
       last_pose.pose.position.z = 0;
       smooth_plan.push_back(last_pose);


      for (const auto &pose : smooth_plan) {
        plan.push_back(pose);
      }

       nav_msgs::Path gui_path;
       gui_path.poses.resize(plan.size());

     if (plan.empty()) {
            gui_path.header.frame_id =global_frame_;
            gui_path.header.stamp = ros::Time::now();
        } else { 
            gui_path.header.frame_id = plan[0].header.frame_id;
            gui_path.header.stamp = plan[0].header.stamp;
        }

        for (unsigned int i = 0; i < plan.size(); i++) {
            gui_path.poses[i] = plan[i];
        }

        global_astar_pub_.publish(gui_path);
        new_global_plan.resize(plan.size());
         for(unsigned int i = 0; i < plan.size(); ++i){
            new_global_plan[i] = plan[i];
          }
    

    
   }
  
}

bool DWAPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!initialized_) {
        ROS_ERROR("DWAPlannerROS has not been initialized.");
        return false;
    }

    goal_reached_ = false;
    rotate = true;
    safe_mode = true;

    ROS_WARN("Start planning.");
    return planner_util_.setPlan(plan);
}

double DWAPlannerROS::triangular_mf(double x, double a, double b, double c) {
    if (x <= a || x > c) return 0;
    if (x <= b && x > a) return (x - a) / (b - a);
    if (x <= c && x > b) return (c - x) / (c - b);
}


std::vector<double> DWAPlannerROS::fuzzify_distance(double distance) {
    std::vector<double> return_;
    torch::Tensor input = torch::tensor({distance_, robot_vel_x, robot_vel_theta}, torch::kFloat);
    torch::Tensor output = fuzzy_nn->forward(input).squeeze(0).clone();

    double a1 = output[0].item<double>();  
    double b1 = a1 + fabs(output[1].item<double>());  
    double c1 = b1 + fabs(output[2].item<double>());  

    return_.push_back(triangular_mf(distance, 0, a1, (2*a1+b1)/3));
    return_.push_back(triangular_mf(distance, (a1+2*b1)/3, b1, (3*b1+c1)/4));
    return_.push_back(triangular_mf(distance, (b1+c1)*2/4, (b1+3*c1)/4, c1));

    ROS_WARN("a1:%f,b1:%f,c1:%f",a1,b1,c1);
    return return_;
}

std::vector<double> DWAPlannerROS::fuzzify_speed(double speed) {
    std::vector<double> return_;
    torch::Tensor input = torch::tensor({speed, robot_vel_x, robot_vel_theta}, torch::kFloat);
    torch::Tensor output = fuzzy_nn->forward(input).squeeze(0).clone();

    double a2 = output[3].item<double>();  
    double b2 = a2 + fabs(output[4].item<double>());  
    double c2 = b2 + fabs(output[5].item<double>());  

    return_.push_back(triangular_mf(speed, 0, a2, (2*a2+b2)/3));  
    return_.push_back(triangular_mf(speed, (a2+2*b2)/3, b2, (3*b2+c2)/4));
    return_.push_back(triangular_mf(speed, (b2+c2)*2/4, (b2+3*c2)/4, c2));
    ROS_WARN("a2:%f,b2:%f,c2:%f",a2,b2,c2);
    return return_;
}

std::vector<double> DWAPlannerROS::fuzzify_angular_velocity(double omega) {
    std::vector<double> return_;
    torch::Tensor input = torch::tensor({omega, robot_vel_x, robot_vel_theta}, torch::kFloat);
    torch::Tensor output = fuzzy_nn->forward(input).squeeze(0).clone();

    double a3 = output[6].item<double>();  
    double b3 = a3 + fabs(output[7].item<double>());  
    double c3 = b3 + fabs(output[8].item<double>()); 

    return_.push_back(triangular_mf(omega, -c3, -(b3+c3)/2, -b3));
    return_.push_back(triangular_mf(omega, -(a3+2*b3)/3, -(2*a3+b3)/3 , -a3));
    return_.push_back(triangular_mf(omega, 0, a3/2, a3));  
    return_.push_back(triangular_mf(omega, (3*a3+b3)/4, (a3+b3)*2/4, (a3+3*b3)/4));
    return_.push_back(triangular_mf(omega, b3, (b3+c3)/2, c3));
    ROS_WARN("a3:%f,b3:%f,c3:%f",a3,b3,c3);
    return return_;
}

void DWAPlannerROS::trainFuzzyNN() {
    
    if (train_inputs.size() < 5) return;  

    std::vector<std::vector<double>> local_train_inputs;
    std::vector<std::vector<double>> local_train_outputs;
    
    {
        std::lock_guard<std::mutex> lock(train_mutex_);
        local_train_inputs = train_inputs;
        local_train_outputs = train_outputs;
    }

    if (local_train_inputs.empty() || local_train_outputs.empty() ||
        local_train_inputs[0].empty() || local_train_outputs[0].empty()) {
        ROS_WARN("trainFuzzyNN: Training data is empty or invalid!");
        return;
    }

    torch::optim::Adam optimizer(fuzzy_nn->parameters(), torch::optim::AdamOptions(0.01));

    for (int epoch = 0; epoch < 500; ++epoch) {
        optimizer.zero_grad();

        std::vector<float> flat_train_inputs;
        for (const auto& row : local_train_inputs) {
            flat_train_inputs.insert(flat_train_inputs.end(), row.begin(), row.end());
        }

        std::vector<float> flat_train_outputs;
        for (const auto& row : local_train_outputs) {
            flat_train_outputs.insert(flat_train_outputs.end(), row.begin(), row.end());
        }

        torch::Tensor inputs = torch::tensor(flat_train_inputs, torch::kFloat).reshape(
            {static_cast<int64_t>(local_train_inputs.size()), static_cast<int64_t>(local_train_inputs[0].size())}
        ).clone();

        torch::Tensor targets = torch::tensor(flat_train_outputs, torch::kFloat).reshape(
            {static_cast<int64_t>(local_train_outputs.size()), static_cast<int64_t>(local_train_outputs[0].size())}
        ).clone();  
/*
        ROS_WARN("Epoch %d: inputs shape: [%ld, %ld]", epoch, inputs.size(0), inputs.size(1));
        ROS_WARN("Epoch %d: targets shape: [%ld, %ld]", epoch, targets.size(0), targets.size(1));
*/
        torch::Tensor outputs = fuzzy_nn->forward(inputs);
      //  ROS_WARN("Epoch %d: outputs shape: [%ld, %ld]", epoch, outputs.size(0), outputs.size(1));

        torch::Tensor loss = torch::mse_loss(outputs, targets);
       // ROS_WARN("Epoch %d: loss: %f", epoch, loss.item<double>());

        if (epoch % 10 == 0) {
            double threshold = 0.1;  
            torch::Tensor diff = torch::abs(outputs - targets);
            torch::Tensor correct_tensor = diff.lt(threshold); 
            double correct = correct_tensor.sum().item<double>();
            double total = diff.numel();
            accuracy = (correct / total) * 100.0;
            std::cout << "Epoch: " << epoch 
                      << ", Loss: " << loss.item<double>() 
                      << ", Accuracy: " << accuracy << "%" 
                      << std::endl;
        }

        loss.backward();
        optimizer.step();
    }
}

void DWAPlannerROS::updateTrainingData(double distance_) {
  
    {
        std::lock_guard<std::mutex> lock(train_mutex_);
        train_inputs.push_back({distance_, robot_vel_x, robot_vel_theta});
    }

    torch::Tensor input;
    {
        std::lock_guard<std::mutex> lock(train_mutex_);
        input = torch::from_blob(
            train_inputs.data(),
            {static_cast<int64_t>(train_inputs.size()), static_cast<int64_t>(train_inputs[0].size())},
            torch::kFloat
        ).clone();
    }
    
    torch::Tensor output = fuzzy_nn->forward(input).detach().cpu().to(torch::kDouble).contiguous();
    auto output_accessor = output.accessor<double, 2>();  

    
    {
        std::lock_guard<std::mutex> lock(train_mutex_);
        int batch_size = output.size(0);
        int output_dim = output.size(1);
        std::vector<double> row;
        for (int j = 0; j < output_dim; j++) {
            row.push_back(output_accessor[batch_size - 1][j]);
        }
        train_outputs.push_back(row);

      
        if (train_inputs.size() > 100) {
            train_inputs.erase(train_inputs.begin());
            train_outputs.erase(train_outputs.begin());
        }
    }
}



bool DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    // Check if plugin is initialized
    if (!initialized_)
    {
        ROS_ERROR("DWAPlannerROS has not been initialized, please call initialize() before using this planner");
        return false;
    }

    // Get the current robot pose
    costmap_ros_->getRobotPose(current_pose_);
    robot_pose_x = current_pose_.pose.position.x;
    robot_pose_y = current_pose_.pose.position.y; 
    robot_pose_theta = tf2::getYaw(current_pose_.pose.orientation);
    planner_->costmap(costmap_);

    // Get the current robot velocity
    geometry_msgs::PoseStamped robot_vel_tf;
    odom_helper_.getRobotVel(robot_vel_tf);

    tf::Pose pose;
    tf::poseMsgToTF(_odom.pose.pose, pose);

    robot_vel_x = _odom.twist.twist.linear.x;
    robot_vel_theta = tf::getYaw(pose.getRotation());

    // Get the transformed global plan
    if(first){
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    transformed_plan.clear();
     if (!planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
        ROS_ERROR("Could not get local plan");
        return false;
     }
     
    // If the plan is empty, return false
     if (transformed_plan.empty()) {
        ROS_WARN("Transformed plan is empty");
        return false;
     }
    
     global_plan_.resize(transformed_plan.size());
     for(unsigned int i = 0; i < transformed_plan.size(); ++i){
        global_plan_[i] = transformed_plan[i];
      }
    }

    geometry_msgs::PoseStamped lookahead_pose;

    if(first){
      lookahead_pose = global_plan_.back(); 
      for (const auto& pose : global_plan_) {
      double dx = pose.pose.position.x - robot_pose_x;
      double dy = pose.pose.position.y - robot_pose_y;
      double distance = hypot(dx, dy);
      if (distance == 0.1) { 
        lookahead_pose = pose;
        break;
       }
      }
    }else if(!first && !new_global_plan.empty()){
        lookahead_pose = new_global_plan.back();  
        for (const auto& pose : new_global_plan) {
            double dx = pose.pose.position.x - robot_pose_x;
            double dy = pose.pose.position.y - robot_pose_y;
            double distance = hypot(dx, dy);
            if (fabs(distance - 0.1) < 0.01) { 
              lookahead_pose = pose;
              break;
             }
            }
    }
    double robot_yaw = robot_pose_theta;

  // Calculate the goal direction from the robot's current pose to the goal pose
    double target_yaw = atan2(lookahead_pose.pose.position.y - robot_pose_y, lookahead_pose.pose.position.x - robot_pose_x); 
    double yaw_error = angles::shortest_angular_distance(robot_yaw, target_yaw);

    /*if (yaw_error > 0.2) {  
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.3;
        return true;
    }else if(yaw_error < -0.2){
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = -0.3;
        return true;
    }
*/
    geometry_msgs::PoseStamped safe_pub;

    for(int i = 0; i < 11; i++){
        geometry_msgs::PoseStamped safe_pose_map;
        safe_pose_map.header.frame_id = "map";
        safe_pose_map.header.stamp = ros::Time::now();
        safe_pose_map.pose.position.x = safes[i][0];
        safe_pose_map.pose.position.y = safes[i][1];
        safe_pose_map.pose.position.z = safes[i][2];
        safe_pose_map.pose.orientation.x = safes[i][3];
        safe_pose_map.pose.orientation.y = safes[i][4];
        safe_pose_map.pose.orientation.z = safes[i][5];
        safe_pose_map.pose.orientation.w = safes[i][6];
        
        geometry_msgs::PoseStamped safe_pose_global;

        // Transform the PoseStamped to global frame
        try{
            safe_pose_global = tf_buffer_.transform(safe_pose_map, global_frame_, ros::Duration(1.0));
        }
        catch(tf2::TransformException &ex){
            ROS_WARN("Transform failed: %s", ex.what());
            continue; 
        } 

        double dx = safe_pose_global.pose.position.x - robot_pose_x;
        double dy = safe_pose_global.pose.position.y - robot_pose_y;

        if(hypot(dx, dy) <= xy_goal_tolerance_){
            safe_pub.pose.position.x = dx;
            safe_pub.pose.position.y = dy;
        }
    }
    

      if(safe_mode){
        safe_pub_.publish(safe_pub);   
      }
     

    // Now proceed with normal DWA planning
    unsigned int start_mx, start_my, goal_mx, goal_my;
    geometry_msgs::PoseStamped goal = global_plan_.back();

    geometry_msgs::PoseStamped current_robot_pose;
    costmap_ros_->getRobotPose(current_robot_pose);   
    double start_wx = current_robot_pose.pose.position.x;
    double start_wy = current_robot_pose.pose.position.y;
    double goal_wx = goal.pose.position.x;
    double goal_wy = goal.pose.position.y;

    if (!costmap_->worldToMap(start_wx, start_wy, start_mx, start_my)){
        ROS_WARN("Cannot convert world current coordinates to map coordinates");
    }

    if(!costmap_->worldToMap(goal_wx, goal_wy, goal_mx, goal_my)) {
        ROS_WARN("Cannot convert world goal coordinates to map coordinates");
    }
    
    // Perform Bresenham's line algorithm to check for obstacles along the straight path
    std::vector<std::pair<int, int>> line_points = planner_->bresenhamLine(start_mx, start_my, goal_mx, goal_my);

    for (const auto& point : line_points) {
        unsigned int mx = point.first;
        unsigned int my = point.second;

        // Get cost from the costmap at each point
        unsigned char cost = costmap_->getCost(mx, my);

        // Check if there's a lethal obstacle
        if (cost == costmap_2d::LETHAL_OBSTACLE) {
            planner_->obstacleFound(true);
            planner_->computeVelocityCommands(robot_vel_x, robot_vel_theta, robot_pose_x, robot_pose_y, robot_pose_theta,
                reference_path, charmap_, size_x_, size_y_,
                resolution, origin_x, origin_y,
                dis_vector, vel_x_vector, vel_theta_vector,
                dwa_cmd_vel_x, dwa_cmd_vel_theta);
            break;
        }
        if(cost == costmap_2d::FREE_SPACE){
            planner_->obstacleFound(false);
            break;
        }
    }

    // Proceed with normal DWA planning
    const unsigned char* charmap = costmap_->getCharMap();

    unsigned int new_size_x = costmap_->getSizeInCellsX();
    unsigned int new_size_y = costmap_->getSizeInCellsY();

    if (charmap_ == nullptr || size_x_ != new_size_x || size_y_ != new_size_y)
    {
        freeMemory();
        size_x_ = new_size_x;
        size_y_ = new_size_y;
        allocateMemory();
    }

    for (unsigned int j = 0; j < size_y_; j++)
    {
        for (unsigned int i = 0; i < size_x_; i++)
            charmap_[i][j] = charmap[i + j * size_x_];
    }
  
    if (first) {
       
        for (const auto& pose : global_plan_)
        {
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;
            double theta = tf2::getYaw(pose.pose.orientation);
            reference_path.emplace_back(std::vector<double>{x, y, theta});
        }

        publishGlobalPlan(global_plan_);   
        first = false;
     }
    else {
        
        for (const auto& pose : new_global_plan)
        {
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;
            double theta = tf2::getYaw(pose.pose.orientation);
            reference_path.emplace_back(std::vector<double>{x, y, theta});
        }
    }

    updateTrainingData(distance_);

    if(accuracy < 0 || distance_ < 0.01){
      dis_vector = {0,0,0};
      vel_x_vector = {0,0,0}; 
      vel_theta_vector = {0,0,0,0,0}; 
    }else{
     dis_vector = fuzzify_distance(distance_);
     vel_x_vector = fuzzify_speed(fabs(robot_vel_x)); 
     vel_theta_vector = fuzzify_angular_velocity(fabs(robot_vel_theta)); 
    }

    bool back = false;
    
 //   ROS_WARN("distance:%f",distance_);
 //   ROS_WARN("robot_vel_x:%f",robot_vel_x);
//    ROS_WARN("robot_vel_theta:%f",robot_vel_theta);
  
    success = planner_->computeVelocityCommands(robot_vel_x, robot_vel_theta, robot_pose_x, robot_pose_y, robot_pose_theta,
            reference_path, charmap_, size_x_, size_y_,
            resolution, origin_x, origin_y, 
            dis_vector, vel_x_vector, vel_theta_vector,
            dwa_cmd_vel_x, dwa_cmd_vel_theta);           
    
    if (!success)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return false;
    }
    else
    {
        if(person_detect){
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
        } else {
            if(distance_ < 0.15 && distance_ > 0.1){    
                cmd_vel.linear.x = dwa_cmd_vel_x + (-1.0)*computeRepulsiveForce(distance_ , min_potential_distance_);
                cmd_vel.angular.z = dwa_cmd_vel_theta; 
                if(distance_ >= 0.15){
                 back = false;  
                }
             }else if(!back){

             if(yaw_error > 0.2){
              cmd_vel.linear.x = dwa_cmd_vel_x;
              cmd_vel.angular.z = fabs(dwa_cmd_vel_theta);
             }else if(yaw_error < 0.2){
               cmd_vel.linear.x = dwa_cmd_vel_x;
               cmd_vel.angular.z = (-1.0)*(fabs(dwa_cmd_vel_theta));
             }
            // Check if goal is reached
            geometry_msgs::PoseStamped robot_pose;
            costmap_ros_->getRobotPose(robot_pose);
            geometry_msgs::PoseStamped global_ = global_plan_.back();

            double dx = robot_pose.pose.position.x - global_.pose.position.x;
            double dy = robot_pose.pose.position.y - global_.pose.position.y;

            if (hypot(dx, dy) <= xy_goal_tolerance_) {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                goal_reached_ = true;  
                rotate = true;
                ROS_INFO("Goal reached.");
            }
            return true;
         }
        }
    }
}

double DWAPlannerROS::computeRepulsiveForce(double distance, double min_potential_distance)
{
  ROS_WARN("in computeRepulsiveForce function");
  return 1.0/2.0 * repulsive_factor_ * (1.0/distance - 1.0/min_potential_distance) * (1.0/distance - 1.0/min_potential_distance);
  
}

bool DWAPlannerROS::isGoalReached()
{
    if (!initialized_)
    {
        ROS_ERROR("DWAPlannerROS has not been initialized, please call initialize() before using this planner");
        return false;
    }
 
    return goal_reached_;
}

void DWAPlannerROS::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
    nav_msgs::Path gui_path;
    gui_path.header.frame_id = map_frame_;
    gui_path.header.stamp = ros::Time::now();
    gui_path.poses = global_plan;
    global_plan_pub_.publish(gui_path);
}

} // namespace dwa_planner_ros
