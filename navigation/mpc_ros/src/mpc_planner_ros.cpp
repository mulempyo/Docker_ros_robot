#include "mpc_planner_ros.h"
#include <pluginlib/class_list_macros.h>

using namespace std;
using namespace Eigen;

PLUGINLIB_EXPORT_CLASS(mpc_ros::MPCPlannerROS, nav_core::BaseLocalPlanner)

namespace mpc_ros{

    MPCPlannerROS::MPCPlannerROS() : costmap_ros_(NULL), rotate(true), tf_(NULL), tf_buffer_(), tf_listener_(tf_buffer_), initialized_(false) 
    {
        ros::NodeHandle nh;
        _nh = nh;
        goal_sub_ = nh.subscribe("/goal", 1, &MPCPlannerROS::goalSub, this);
        global_timer_ = nh.createTimer(ros::Duration(0.5), &MPCPlannerROS::globalReplanning, this);
    }
	MPCPlannerROS::MPCPlannerROS(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), rotate(true), tf_(NULL), tf_buffer_(), tf_listener_(tf_buffer_), initialized_(false) {}

	MPCPlannerROS::~MPCPlannerROS() {
          
       }

	void MPCPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){

        ros::NodeHandle private_nh("~/" + name);

		tf_ = tf;
		costmap_ros_ = costmap_ros;
        //initialize the copy of the costmap the controller will use
        costmap_ = costmap_ros_->getCostmap();
        global_frame_ = costmap_ros_->getGlobalFrameID();
        robot_base_frame_ = costmap_ros_->getBaseFrameID();
        footprint_spec_ = costmap_ros_->getRobotFootprint();
        size_x_ = costmap_->getSizeInCellsX();
        
        planner_util_.initialize(tf_, costmap_, costmap_ros_->getGlobalFrameID());
        astar.initialize("mpc_ros", costmap_ros_);
        
        if( private_nh.getParam( "odom_frame", _odom_frame ))
        {
            odom_helper_.setOdomTopic( _odom_frame );
        }

        std::string controller_frequency_param_name;
        double controller_frequency = 0;
        if(!_nh.searchParam("move_base/controller_frequency", controller_frequency_param_name)) {
            ROS_WARN("controller_frequency_param_name doesn't exits");
        } else {
            _nh.param(controller_frequency_param_name, controller_frequency, 20.0);
            
            if(controller_frequency > 0) {
            } else {
                ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
            }
        }
        
        _dt = double(1.0/controller_frequency); 

        //Parameter for topics & Frame name
        private_nh.param<std::string>("map_frame", _map_frame, "map" ); 
        private_nh.param<std::string>("odom_frame", _odom_frame, "odom");
        private_nh.param<std::string>("base_frame", _base_frame, "base_footprint");

        private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.2);


        //Publishers and Subscribers
        _sub_odom   = _nh.subscribe("/odometry/filtered", 1, &MPCPlannerROS::odomCB, this);
        global_plan_pub_   = _nh.advertise<nav_msgs::Path>("mpc_planner", 1);
        _pub_mpctraj = _nh.advertise<nav_msgs::Path>("mpc_trajectory",1);
        _pub_odompath = _nh.advertise<nav_msgs::Path>("mpc_reference",1);
        global_astar_pub_ = private_nh.advertise<nav_msgs::Path>("mpc_astar_plan", 1);

        //Init variables
        _throttle = 0.0; 
        _w = 0.0;
        _speed = 0.0;

        _twist_msg = geometry_msgs::Twist();
        _mpc_traj = nav_msgs::Path();

        
        dsrv_ = new dynamic_reconfigure::Server<MPCPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<MPCPlannerConfig>::CallbackType cb = boost::bind(&MPCPlannerROS::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        first = true;
        once = true;
        initialized_ = true;
    }

  void MPCPlannerROS::reconfigureCB(MPCPlannerConfig &config, uint32_t level) {
      // update generic local planner params
      base_local_planner::LocalPlannerLimits limits;
      limits.max_vel_trans = config.max_vel_trans;
      limits.min_vel_trans = config.min_vel_trans;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_vel_theta = config.max_vel_theta;
      limits.min_vel_theta = config.min_vel_theta;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_lim_trans = config.acc_lim_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.theta_stopped_vel = config.theta_stopped_vel;

      //Parameter for MPC solver
      _debug_info = config.debug_info;
      _delay_mode = config.delay_mode;
      _max_speed = config.max_speed;
      _waypointsDist = config.waypoints_dist;
      _pathLength = config.path_length;
      _mpc_steps = config.steps;
      _ref_cte = config.ref_cte;
      _ref_vel = config.ref_vel;
      _ref_etheta = config.ref_etheta;
      _w_cte = config.w_cte;
      _w_etheta = config.w_etheta;
      _w_vel = config.w_vel;
      _w_angvel = config.w_angvel;
      _w_angvel_d = config.w_angvel_d;
      _w_accel_d = config.w_accel_d;
      _w_accel = config.w_accel;
      _max_angvel = config.max_angvel;
      _max_throttle = config.max_throttle;
      _bound_value = config.bound_value;

  }

  void MPCPlannerROS::goalSub(geometry_msgs::PoseStamped goal){
    try {
        if(once){
            goal_ = tf_buffer_.transform(goal, global_frame_, ros::Duration(1.0));
        }
        once = false;
        geometry_msgs::PoseStamped start;

        goal_transformed_ = true;
    } catch(tf2::TransformException &ex) {
        ROS_WARN("Goal transform failed: %s", ex.what());
        goal_transformed_ = false;
    }
}

void MPCPlannerROS::globalReplanning(const ros::TimerEvent& event){
    if(!first){
    new_global_plan.clear();
    std::vector<geometry_msgs::PoseStamped> plan;
    geometry_msgs::PoseStamped goal,current;
    costmap_ros_->getRobotPose(current);
    start = current;
    plan.clear();

    if(goal_transformed_){
        goal = goal_;
    }

    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
      ROS_WARN("The start is out of the map bounds.");
      start_x = std::round(start.pose.position.x / costmap_->getResolution());
      start_y = std::round(start.pose.position.y / costmap_->getResolution());
     }

    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
        ROS_WARN("The goal is out of the map bounds.");
    }

    path = astar.cudaAStarSearch(start_x, start_y, goal_x, goal_y);

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

      int poly_order = 5;
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
  
	bool MPCPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
        if( ! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        goal_reached_ = false;

        ROS_WARN("start Plan");

        //Init parameters for MPC object
        _mpc_params["DT"] = _dt;
        //_mpc_params["LF"] = _Lf;
        _mpc_params["STEPS"]    = _mpc_steps;
        _mpc_params["REF_CTE"]  = _ref_cte;
        _mpc_params["REF_ETHETA"] = _ref_etheta;
        _mpc_params["REF_V"]    = _ref_vel;
        _mpc_params["W_CTE"]    = _w_cte;
        _mpc_params["W_EPSI"]   = _w_etheta;
        _mpc_params["W_V"]      = _w_vel;
        _mpc_params["W_ANGVEL"]  = _w_angvel;
        _mpc_params["W_A"]      = _w_accel;
        _mpc_params["W_DANGVEL"] = _w_angvel_d;
        _mpc_params["W_DA"]     = _w_accel_d;
        _mpc_params["LINVEL"]   = _max_speed;
        _mpc_params["ANGVEL"]   = _max_angvel;
        _mpc_params["MAXTHR"]   = _max_throttle;
        _mpc_params["BOUND"]    = _bound_value;
        _mpc.LoadParams(_mpc_params);
        //Display the parameters
        cout << "\n===== Parameters =====" << endl;
        cout << "debug_info: "  << _debug_info << endl;
        cout << "delay_mode: "  << _delay_mode << endl;
        //cout << "vehicle_Lf: "  << _Lf << endl;
        cout << "frequency: "   << _dt << endl;
        cout << "mpc_steps: "   << _mpc_steps << endl;
        cout << "mpc_ref_vel: " << _ref_vel << endl;
        cout << "mpc_w_cte: "   << _w_cte << endl;
        cout << "mpc_w_etheta: "  << _w_etheta << endl;
        cout << "mpc_max_angvel: "  << _max_angvel << endl;

        planner_util_.setPlan(orig_global_plan);
        
	return true;
    }

    bool MPCPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
   
    if (!costmap_ros_->getRobotPose(current_pose_)) {
        ROS_ERROR("Could not get robot pose");
        return false;
    }

    double robot_pose_x = current_pose_.pose.position.x;
    double robot_pose_y = current_pose_.pose.position.y;
    double robot_pose_theta = tf2::getYaw(current_pose_.pose.orientation);

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if (!planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
        ROS_ERROR("Could not get local plan");
        return false;
    }

    if (transformed_plan.empty()) {
        ROS_WARN_NAMED("mpc_planner", "Received an empty transformed plan.");
        return false;
    }
   
    global_plan_.resize(transformed_plan.size());
        for (unsigned int i = 0; i < transformed_plan.size(); ++i) {
            global_plan_[i] = transformed_plan[i];
        }

    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    geometry_msgs::PoseStamped lookahead_pose,nearest_pose,last;
    double distance;

    if (first) {
        for (const auto& pose : global_plan_) {
            double dx = pose.pose.position.x - robot_pose_x;
            double dy = pose.pose.position.y - robot_pose_y;
            distance = hypot(dx, dy);
    
            if (distance < 0.3 && distance >= 0.1) {
                nearest_pose = pose; 
                break;
            }
        }
    } else if(!first && !new_global_plan.empty()) {
        for (const auto& pose : new_global_plan) {
            double dx = pose.pose.position.x - robot_pose_x;
            double dy = pose.pose.position.y - robot_pose_y;
            distance = hypot(dx, dy);

            if (distance < 0.3 && distance >= 0.1) {
                nearest_pose = pose; 
                break;
            }
        }
    }

    double angle_to_goal;
    last = global_plan_.back();

    if(hypot(last.pose.position.x - robot_pose_x, last.pose.position.y - robot_pose_y) < 0.25){
       lookahead_pose.pose.position.x = last.pose.position.x;
       lookahead_pose.pose.position.y = last.pose.position.y;
       lookahead_pose.pose.position.z = 0.0;
       del_x = last.pose.position.x - robot_pose_x;
    }else{
       double dx = nearest_pose.pose.position.x - robot_pose_x;
       double dy = nearest_pose.pose.position.y - robot_pose_y;
       del_x = dx;
       angle_to_goal = atan2(dy, dx); 

       lookahead_pose.pose.position.x = robot_pose_x + distance * cos(angle_to_goal);
       lookahead_pose.pose.position.y = robot_pose_y + distance * sin(angle_to_goal);
       lookahead_pose.pose.position.z = 0.0;
    }
    
    tf2::Quaternion q;
    q.setRPY(0, 0, angle_to_goal);
    lookahead_pose.pose.orientation = tf2::toMsg(q);

    double target_yaw;
    double robot_yaw = robot_pose_theta;
    target_yaw = atan2(lookahead_pose.pose.position.y - robot_pose_y, lookahead_pose.pose.position.x - robot_pose_x); 

    yaw_error = angles::shortest_angular_distance(robot_yaw, target_yaw);

    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();

    bool success = mpcComputeVelocityCommands(current_pose_, robot_vel, drive_cmds);

         if(!success)
         {
          cmd_vel.linear.x = 0.0;
          cmd_vel.angular.z = 0.0;
          return false;
         } else{

         cmd_vel.linear.x = drive_cmds.pose.position.x;
         cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

         std::vector<geometry_msgs::PoseStamped> local_plan;

          for(unsigned int i = 0; i < result_traj_.getPointsSize(); ++i) {
            double p_x, p_y, p_th;
            result_traj_.getPoint(i, p_x, p_y, p_th);

            geometry_msgs::PoseStamped p;
            p.header.frame_id = costmap_ros_->getGlobalFrameID();
            p.header.stamp = ros::Time::now();
            p.pose.position.x = p_x;
            p.pose.position.y = p_y;
            p.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, p_th);
            tf2::convert(q, p.pose.orientation);
            local_plan.push_back(p);
        }
         publishGlobalPlan(local_plan);
    
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
           once = true;
           ROS_INFO("Goal reached.");
         }
          return true;
         }
       
     }

    // Timer: Control Loop (closed loop nonlinear MPC)
    bool MPCPlannerROS::mpcComputeVelocityCommands(geometry_msgs::PoseStamped global_pose, geometry_msgs::PoseStamped& global_vel, 
    geometry_msgs::PoseStamped& drive_cmds)
    {   
        base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
        geometry_msgs::PoseStamped goal_pose = global_plan_.back();
        result_traj_.cost_ = 1;
   
        nav_msgs::Odometry base_odom = _odom;

        const double px = base_odom.pose.pose.position.x; 
        const double py = base_odom.pose.pose.position.y;
        tf::Pose pose;
        tf::poseMsgToTF(base_odom.pose.pose, pose);
        double theta = tf::getYaw(pose.getRotation());
        const double v = base_odom.twist.twist.linear.x; 
     
        const double w = _w; 
     
        const double throttle = _throttle; 
        const double dt = _dt;
   
        nav_msgs::Path odom_path = nav_msgs::Path();
        
        try{
        if(first){
         odom_path.poses.clear();
         for (size_t i = 0; i < global_plan_.size(); i++)
         {
          geometry_msgs::PoseStamped transformed_pose;
          transformed_pose = tf_buffer_.transform(global_plan_[i], _odom_frame, ros::Duration(1.0));
          odom_path.poses.push_back(transformed_pose);
         }
         if (!odom_path.poses.empty())
         {
          geometry_msgs::PoseStamped last_global = global_plan_.back();
          geometry_msgs::PoseStamped last_odom = tf_buffer_.transform(last_global, _odom_frame, ros::Duration(1.0));
          odom_path.poses.back() = last_odom;
         }

         if (odom_path.poses.size() > 0)
         {
          odom_path.header.frame_id = _odom_frame;
          odom_path.header.stamp = ros::Time::now();
          _pub_odompath.publish(odom_path);
         }
         else
         {
          ROS_DEBUG_NAMED("mpc_ros", "Odom path is empty.");
          return false;
         }
        }else{
         odom_path.poses.clear();
         for (size_t i = 0; i < new_global_plan.size(); i++)
         {
          geometry_msgs::PoseStamped transformed_pose;
          transformed_pose = tf_buffer_.transform(new_global_plan[i], _odom_frame, ros::Duration(1.0));
          odom_path.poses.push_back(transformed_pose);
         }
         if (!odom_path.poses.empty())
         {
          geometry_msgs::PoseStamped last_global = new_global_plan.back();
          geometry_msgs::PoseStamped last_odom = tf_buffer_.transform(last_global, _odom_frame, ros::Duration(1.0));
          odom_path.poses.back() = last_odom;
         }

         if (odom_path.poses.size() > 0)
         {
          odom_path.header.frame_id = _odom_frame;
          odom_path.header.stamp = ros::Time::now();
          _pub_odompath.publish(odom_path);
         }
         else
         {
          ROS_DEBUG_NAMED("mpc_ros", "Odom path is empty.");
          return false;
         }
        }
        }
        catch(tf2::TransformException &ex)
        {
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
          return false;
        }
        first = false;
        const int N = odom_path.poses.size(); 
        const double costheta = cos(theta);
        const double sintheta = sin(theta);

        VectorXd x_veh(N);
        VectorXd y_veh(N);
        for(int i = 0; i < N; i++) 
        {
            const double dx = odom_path.poses[i].pose.position.x - px;
            const double dy = odom_path.poses[i].pose.position.y - py;
            x_veh[i] = dx * costheta + dy * sintheta;
            y_veh[i] = dy * costheta - dx * sintheta;
 
        }
        
        auto coeffs = polyfit(x_veh, y_veh, 5); 
        double cte  = polyeval(coeffs, 0.0);
        
        if (std::isnan(cte) || std::isinf(cte)) {
            ROS_ERROR("Computed cte is NaN or Inf! Setting to 0.");
            cte = 0.0;
        }
 
        double etheta = yaw_error;

        double gx = 0;
        double gy = 0;
        int N_sample = N * 0.3;
        for(int i = 1; i < N_sample; i++) 
        {
            gx += odom_path.poses[i].pose.position.x - odom_path.poses[i-1].pose.position.x;
            gy += odom_path.poses[i].pose.position.y - odom_path.poses[i-1].pose.position.y;
        }   

        double temp_theta = theta;
        double traj_deg = atan2(gy,gx);
        double PI = 3.141592;
      
        if(temp_theta <= -PI + traj_deg){
            temp_theta = temp_theta + 2 * PI;
        }
        
        double theta_diff = temp_theta - traj_deg;
        double max_theta_diff = PI / 6; 
        double min_theta_diff = -PI / 6; 
     
        if (theta_diff > max_theta_diff) {
          theta_diff = max_theta_diff;
        } else if (theta_diff < min_theta_diff) {
          theta_diff = min_theta_diff;
        } 

        if(theta_diff > 0){
            etheta -= theta_diff;  
          } else{
            etheta += theta_diff;
          }          
       
        VectorXd state(6); 
        const double theta_act = w * dt;
        const double px_act = v * dt;
        const double py_act = 0;
        const double v_act = v + throttle * dt;
        double cte_act = cte+v*sin(etheta)*dt;
        const double etheta_act = etheta - theta_act;  
            
        state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;

        ROS_INFO("MPC State: px=%.3f, py=%.3f, theta=%.3f, v=%.3f, cte=%.3f, etheta=%.3f", 
            state[0], state[1], state[2], state[3], state[4], state[5]);

        vector<double> mpc_results = _mpc.Solve(state, coeffs);    
        
        _w = mpc_results[0]; // radian/sec, angular velocity
        _throttle = mpc_results[1]; // acceleration
          
        
        _speed = v + _throttle * dt;  // speed
        if (_speed >= _max_speed)
            _speed = _max_speed;
        if(_speed <= 0.0)
            _speed = 0.0;

        // Display the MPC predicted trajectory
        _mpc_traj = nav_msgs::Path();
        _mpc_traj.header.frame_id = _base_frame; // points in car coordinate        
        _mpc_traj.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped tempPose;
        tf2::Quaternion myQuaternion;
        for(int i=0; i<_mpc.mpc_x.size(); i++)
        {
            tempPose.header = _mpc_traj.header;
            tempPose.pose.position.x = _mpc.mpc_x[i];
            tempPose.pose.position.y = _mpc.mpc_y[i];
            myQuaternion.setRPY( 0, 0, _mpc.mpc_theta[i] );  
            tempPose.pose.orientation.x = myQuaternion[0];
            tempPose.pose.orientation.y = myQuaternion[1];
            tempPose.pose.orientation.z = myQuaternion[2];
            tempPose.pose.orientation.w = myQuaternion[3];
                
            _mpc_traj.poses.push_back(tempPose); 
        }   

        if(result_traj_.cost_ < 0){
            drive_cmds.pose.position.x = 0;
            drive_cmds.pose.position.y = 0;
            drive_cmds.pose.position.z = 0;
            drive_cmds.pose.orientation.w = 1;
            drive_cmds.pose.orientation.x = 0;
            drive_cmds.pose.orientation.y = 0;
            drive_cmds.pose.orientation.z = 0;
        }
        else{
            drive_cmds.pose.position.x = _speed;
            drive_cmds.pose.position.y = 0;
            drive_cmds.pose.position.z = 0;
            tf2::Quaternion q;
            q.setRPY(0, 0, _w);
            tf2::convert(q, drive_cmds.pose.orientation);
        }
        _pub_mpctraj.publish(_mpc_traj);

        if(goal_reached_){
            odom_path.poses.clear();
            ROS_INFO("Path cleared after goal reached.");
        }
        return true;
    }


    bool MPCPlannerROS::isGoalReached()
    {
      // check if plugin is initialized
      if (!initialized_)
      {
        ROS_ERROR("mpc_local_planner has not been initialized, please call initialize() before using this planner");
        return false;
      }
        return goal_reached_;
    }

    // Evaluate a polynomial.
    double MPCPlannerROS::polyeval(Eigen::VectorXd coeffs, double x) 
    {
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++) 
        {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }

Eigen::VectorXd MPCPlannerROS::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {

    if (xvals.size() != yvals.size()) {
        ROS_ERROR("polyfit failed: xvals.size() != yvals.size()");
    }

    if (xvals.size() == 0) {
        ROS_ERROR("polyfit failed: xvals.size() == 0");
    }

    if (order >= xvals.size()) {
        int poly_order = std::min(5, int(xvals.size() - 1));
        order = poly_order;
        ROS_WARN("polyfit failed: xvals.size() == 0, order:%d",order);
    }

    ROS_WARN("polyfit order: %d", order);

    ROS_WARN("Before polyfit: x_vals size = %ld, y_vals size = %ld", xvals.size(), yvals.size());
    for (size_t i = 0; i < xvals.size(); i++) {
    ROS_WARN("x[%ld] = %.3f, y[%ld] = %.3f", i, xvals[i], i, yvals[i]);
    }

    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);
    
    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

        for (int j = 1; j <= order; j++) {  
            for (int i = 0; i < xvals.size(); i++) {
                A(i, j) = A(i, j - 1) * xvals(i); 
            }
        }
        

    auto Q = A.householderQr();
    Eigen::VectorXd result = Q.solve(yvals);

    for (int i = 0; i < result.size(); i++) {
        if (std::isnan(result[i]) || std::isinf(result[i])) {
            ROS_ERROR("polyfit output contains NaN/Inf at index %d!", i);
            return Eigen::VectorXd::Zero(1);
        }
    }

    return result;
}
    
    // CallBack: Update odometry
    void MPCPlannerROS::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        _odom = *odomMsg;
    }


    void MPCPlannerROS::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
    {
      nav_msgs::Path gui_path;
      gui_path.header.frame_id = _map_frame;
      gui_path.header.stamp = ros::Time::now();
      gui_path.poses = global_plan;
      global_plan_pub_.publish(gui_path);
    }
}
