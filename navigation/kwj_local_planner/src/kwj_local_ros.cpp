/*
  TEB (Timed Elastic Band) 로컬 플래너는 전역(planning)으로 생성된 경로를 시간(time) 정보가 포함된 일종의 “탄성 밴드(elastic band)”로 보고, 
  이를 곡선 최적화(graph-based optimization) 기법으로 실시간에 가깝게 다듬어 주는 로컬 플래너입니다. 시간정보를 적용하는 부분을 따왔습니다.
*/

#include "kwj_ros.h"
#include <pluginlib/class_list_macros.h>

using namespace std;
using namespace Eigen;

PLUGINLIB_EXPORT_CLASS(kwj_local_planner::KWJPlannerROS, nav_core::BaseLocalPlanner)

namespace kwj_local_planner{

    KWJPlannerROS::KWJPlannerROS() : costmap_ros_(NULL), tf_(NULL), tf_buffer_(), tf_listener_(tf_buffer_), initialized_(false) 
    {
        ros::NodeHandle nh;
        _nh = nh;
    }
	KWJPlannerROS::KWJPlannerROS(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), tf_buffer_(), tf_listener_(tf_buffer_), initialized_(false) {
        ros::NodeHandle nh;
        _nh = nh;
    }

	KWJPlannerROS::~KWJPlannerROS() {
        if (world_model_) {
        delete world_model_;
    }
    }

	void KWJPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){

    ros::NodeHandle private_nh("~/" + name);

	tf_ = tf;
	costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    global_frame_ = costmap_ros_->getGlobalFrameID();
    robot_base_frame_ = costmap_ros_->getBaseFrameID();
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    size_x_ = costmap_->getSizeInCellsX();
    width      = costmap_->getSizeInCellsX();
    height     = costmap_->getSizeInCellsY();
    resolution       = costmap_->getResolution();
    origin_x         = costmap_->getOriginX();
    origin_y         = costmap_->getOriginY();
        
    planner_util_.initialize(tf_, costmap_, costmap_ros_->getGlobalFrameID());
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
        
    if( private_nh.getParam( "odom_frame", _odom_frame ))
    {
        odom_helper_.setOdomTopic( _odom_frame );
    }

    private_nh.param<std::string>("map_frame", _map_frame, "map" ); 
    private_nh.param<std::string>("odom_frame", _odom_frame, "odom");
    private_nh.param<std::string>("base_frame", _base_frame, "base_footprint");

    private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.2);

    _sub_odom   = private_nh.subscribe("/odometry/filtered", 1, &KWJPlannerROS::odomCB, this);
    global_plan_pub_   = private_nh.advertise<nav_msgs::Path>("kwj_planner", 1);
    odom_plan_pub_ = private_nh.advertise<nav_msgs::Path>("kwj_odom_planner", 1);
    marker_array_pub = private_nh.advertise<visualization_msgs::MarkerArray>("odom_point_markers", 1);
    _pub_kwjtraj = private_nh.advertise<nav_msgs::Path>("kwj_trajectory",1);
    _pub_odompath = private_nh.advertise<nav_msgs::Path>("kwj_reference",1);

    std::string controller_frequency_param_name;
    double controller_frequency = 0;
        if(!private_nh.searchParam("move_base/controller_frequency", controller_frequency_param_name)) {
            ROS_WARN("controller_frequency_param_name doesn't exits");
        } else {
            private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
            
            if(controller_frequency < 0) {
                ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
            }
        }
    _dt = double(1.0/controller_frequency);

    std::string inflation_radius_param_name;
    if(!private_nh.searchParam("move_base/inflation_radius", inflation_radius_param_name)) {
        ROS_WARN("inflation_radius_param_name doesn't exits");
    } else {
        private_nh.param(inflation_radius_param_name, inflation_radius, 0.2);
        
        if(inflation_radius < 0) {
            ROS_WARN("A inflation_radius less than 0");
        }
    }

    _a = 0.0; 
    _w = 0.0;
    _speed = 0.0;
    _kwj_traj = nav_msgs::Path();

        
    dsrv_ = new dynamic_reconfigure::Server<KWJPlannerConfig>(private_nh);
    dynamic_reconfigure::Server<KWJPlannerConfig>::CallbackType cb = boost::bind(&KWJPlannerROS::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    initialized_ = true;
    }

  void KWJPlannerROS::reconfigureCB(KWJPlannerConfig &config, uint32_t level) {
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

      _debug_info = config.debug_info;
      _delay_mode = config.delay_mode;
      _max_speed = config.max_speed;
      _kwj_steps = config.steps;
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
      _max_a = config.max_a;
      _bound_value = config.bound_value;
      _t_max = config.t_max;
      _dt_min = config.dt_min;
      _dt_max = config.dt_max;
      _o_min = config.o_min;
      _o_weight = config.o_weight;

  }
  
	bool KWJPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
        if(!initialized_) {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        goal_reached_ = false;

        ROS_WARN("start Plan");
     
        _kwj_params["DT"] = _dt;
        _kwj_params["STEPS"]    = _kwj_steps;
        _kwj_params["REF_CTE"]  = _ref_cte;
        _kwj_params["REF_ETHETA"] = _ref_etheta;
        _kwj_params["REF_V"]    = _ref_vel;
        _kwj_params["W_CTE"]    = _w_cte;
        _kwj_params["W_EPSI"]   = _w_etheta;
        _kwj_params["W_V"]      = _w_vel;
        _kwj_params["W_ANGVEL"]  = _w_angvel;
        _kwj_params["W_A"]      = _w_accel;
        _kwj_params["W_DANGVEL"] = _w_angvel_d;
        _kwj_params["W_DA"]     = _w_accel_d;
        _kwj_params["LINVEL"]   = _max_speed;
        _kwj_params["ANGVEL"]   = _max_angvel;
        _kwj_params["MAXTHR"]   = _max_a;
        _kwj_params["BOUND"]    = _bound_value;
        _kwj_params["T_MAX"]    = _t_max;
        _kwj_params["DT_MIN"]    = _dt_min;
        _kwj_params["DT_MAX"]    = _dt_max;
        _kwj_params["OBSTACLE_MIN"]    = _o_min;
        _kwj_params["OBSTACLE_WEIGHT"]    = _o_weight;
        
        _kwj.LoadParams(_kwj_params);

        planner_util_.setPlan(orig_global_plan);
        
	    return true;
    }

    bool KWJPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
   
    if (!costmap_ros_->getRobotPose(current_pose_)) {
        ROS_ERROR("Could not get robot pose");
        return false;
    }

    costmap_ros_->updateMap();
    costmap_ = costmap_ros_->getCostmap();

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if (!planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
        ROS_ERROR("Could not get local plan");
        return false;
    }

    if (transformed_plan.empty()) {
        ROS_WARN_NAMED("kwj_planner", "Received an empty transformed plan.");
        return false;
    }
   
    global_plan_.resize(transformed_plan.size());
        for (unsigned int i = 0; i < transformed_plan.size(); ++i) {
            global_plan_[i] = transformed_plan[i];
        }

    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();

    bool success = kwjComputeVelocityCommands(current_pose_, robot_vel, drive_cmds);

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
           ROS_INFO("Goal reached.");
         }
          return true;
         }
       
     }

    bool KWJPlannerROS::kwjComputeVelocityCommands(geometry_msgs::PoseStamped global_pose, geometry_msgs::PoseStamped& global_vel, 
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
        const double a = _a; 
        const double dt = _dt;
   
        odom_path.poses.clear();

        geometry_msgs::PoseStamped current_pose;
        if (!costmap_ros_->getRobotPose(current_pose)) {
          ROS_ERROR("Could not get robot pose");
          return false;
        }

        geometry_msgs::PoseStamped new_pt_odom;
        nav_msgs::Path path_point;
        path_point.poses.clear();

        geometry_msgs::PoseStamped current_pose_odom = tf_buffer_.transform(current_pose, _odom_frame, ros::Duration(1.0));
        double robot_pose_x = current_pose_odom.pose.position.x;
        double robot_pose_y = current_pose_odom.pose.position.y;
        double robot_pose_theta = tf2::getYaw(current_pose_odom.pose.orientation);

        odom_plan.clear();
        odom_plan.resize(global_plan_.size());
        for (unsigned int i = 0; i < global_plan_.size(); ++i) {
                odom_plan[i] = tf_buffer_.transform(global_plan_[i], _odom_frame, ros::Duration(1.0));
            }

        _kwj.currentPose(robot_pose_x, robot_pose_y, robot_pose_theta);

        costmap_ros_->updateMap();
        costmap_ = costmap_ros_->getCostmap();
        unsigned int width = costmap_->getSizeInCellsX();
        unsigned int height = costmap_->getSizeInCellsY();
        double origin_x = costmap_->getOriginX();
        double origin_y = costmap_->getOriginY();
        double resolution = costmap_->getResolution();

        obstacles_.clear();
        for (unsigned int iy = 0; iy < height; ++iy) {
            for (unsigned int ix = 0; ix < width; ++ix) {
                unsigned char cost = costmap_->getCost(ix, iy);
                if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                    double wx = origin_x + (ix + 0.5) * resolution;
                    double wy = origin_y + (iy + 0.5) * resolution;
                    obstacles_.emplace_back(wx, wy);
                }
            }
        }    
        
        geometry_msgs::PoseStamped final_goal = global_plan_.back();
        geometry_msgs::PoseStamped final_goal_odom = tf_buffer_.transform(final_goal, _odom_frame, ros::Duration(0.1));
        double robot_yaw = tf2::getYaw(current_pose_odom.pose.orientation);

        if (!global_plan_.empty()) {
    
            bool found = false;
        
            for (const auto& pose : odom_plan) {

                double dx = pose.pose.position.x - current_pose_odom.pose.position.x;
                double dy = pose.pose.position.y - current_pose_odom.pose.position.y;
                double distance = hypot(dx, dy);

                double forward_x = cos(robot_yaw);
                double forward_y = sin(robot_yaw);
                double dot_product = dx * forward_x + dy * forward_y; 

                if (dot_product < 0.0) continue;
        
                if (fabs(distance - 2.0) < 0.05) {
                    new_pt_odom = pose;
                    double yaw = std::atan2(dy, dx);
                    tf2::Quaternion q;
                    q.setRPY(0, 0, yaw);
                    new_pt_odom.pose.orientation = tf2::toMsg(q);

                    path_point.poses.clear();
                    path_point.poses.push_back(new_pt_odom);
                    markPublish(path_point); 

                    makeReference(new_pt_odom, current_pose_odom);

                    found = true;
                    break;
                }
            }
        
            if (!found) { //found is false, if found is not true, found = false, (!found) is true.
                ROS_WARN("last");
                new_pt_odom = final_goal_odom;
                double dx = new_pt_odom.pose.position.x - current_pose_odom.pose.position.x;
                double dy = new_pt_odom.pose.position.y - current_pose_odom.pose.position.y;
                double yaw = std::atan2(dy, dx);
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                new_pt_odom.pose.orientation = tf2::toMsg(q);

                path_point.poses.clear();
                path_point.poses.push_back(new_pt_odom);
                markPublish(path_point); 

                makeReference(new_pt_odom, current_pose_odom);

            }
        
        }

        if (!odom_path.poses.empty()) {
            size_t mid_idx = odom_path.poses.size() / 2;
            geometry_msgs::PoseStamped mid_pt = odom_path.poses[mid_idx];
            double dx = mid_pt.pose.position.x - current_pose_odom.pose.position.x;
            double dy = mid_pt.pose.position.y - current_pose_odom.pose.position.y;
            double dist = std::hypot(dx, dy);

            if (new_pt_odom == final_goal_odom) {
                dx = new_pt_odom.pose.position.x - current_pose_odom.pose.position.x;
                dy = new_pt_odom.pose.position.y - current_pose_odom.pose.position.y;
            }

            double target_yaw = std::atan2(dy, dx);
            yaw_error = angles::shortest_angular_distance(robot_yaw, target_yaw);
            } else {
                ROS_DEBUG("odom_path is empty; skipping yaw_error update.");
            }
    
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

        costmap_ros_->updateMap();
        costmap_ = costmap_ros_->getCostmap();
        obstacles_.clear();
        for (unsigned int iy = 0; iy < height; ++iy) {
            for (unsigned int ix = 0; ix < width; ++ix) {
                unsigned char cost = costmap_->getCost(ix, iy);
                if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                    double wx = origin_x + (ix + 0.5) * resolution;
                    double wy = origin_y + (iy + 0.5) * resolution;
                    obstacles_.emplace_back(wx, wy);
                }
            }
        }   

        _kwj.obstacle(obstacles_);
 
        double etheta = yaw_error;

        double gx = 0;
        double gy = 0;
        int N_sample = N;
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
        double max_theta_diff = PI / 4; 
        double min_theta_diff = -PI / 4; 
     
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
        const double px_act = v * dt;
        const double py_act = 0;
        const double theta_act = w * dt;
        const double v_act = v + a * dt;
        double cte_act = cte+v*sin(etheta)*dt;
        const double etheta_act = etheta - theta_act;  
            
        state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;

        ROS_INFO("KWJ State: px=%.3f, py=%.3f, theta=%.3f, v=%.3f, cte=%.3f, etheta=%.3f", 
            state[0], state[1], state[2], state[3], state[4], state[5]);
       
        vector<double> ctrl = _kwj.Solve(state, coeffs);    
        
        _w = ctrl[0]; // radian/sec, angular velocity
        _a = ctrl[1]; // acceleration

        _kwj_traj = nav_msgs::Path();
        _kwj_traj.header.frame_id = _base_frame;      
        _kwj_traj.header.stamp = ros::Time::now();
        _kwj_traj.poses.clear();
        for (size_t i = 0; i < _kwj.kwj_x.size(); ++i) {
            geometry_msgs::PoseStamped p;
            p.header = _kwj_traj.header;
            p.pose.position.x = _kwj.kwj_x[i];
            p.pose.position.y = _kwj.kwj_y[i];
            tf2::Quaternion tq;
            tq.setRPY(0, 0, _kwj.kwj_theta[i]);
            tf2::convert(tq, p.pose.orientation);
            _kwj_traj.poses.push_back(p);
           }
        _pub_kwjtraj.publish(_kwj_traj);

        if(result_traj_.cost_ < 0){
            drive_cmds.pose.position.x = 0;
            drive_cmds.pose.position.y = 0;
            drive_cmds.pose.position.z = 0;
            drive_cmds.pose.orientation.w = 1;
            drive_cmds.pose.orientation.x = 0;
            drive_cmds.pose.orientation.y = 0;
            drive_cmds.pose.orientation.z = 0;
        }else{
            for (size_t i = 0; i < _kwj.kwj_time.size(); ++i){
                _speed = v + _a * _kwj.kwj_time[i];
                if(_speed <= 0.0) {_speed = 0.0;}
                drive_cmds.pose.position.x = _speed;
                tf2::Quaternion q;
                q.setRPY(0,0,_w);
                tf2::convert(q, drive_cmds.pose.orientation);

            }
        }

        if(goal_reached_){
            odom_path.poses.clear();
            ROS_INFO("Path cleared after goal reached.");
        }
        return true;
    }


    bool KWJPlannerROS::isGoalReached()
    {
      // check if plugin is initialized
      if (!initialized_)
      {
        ROS_ERROR("kwj_local_planner has not been initialized, please call initialize() before using this planner");
        return false;
      }
        return goal_reached_;
    }

    // Evaluate a polynomial.
    double KWJPlannerROS::polyeval(Eigen::VectorXd coeffs, double x) 
    {
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++) 
        {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }

Eigen::VectorXd KWJPlannerROS::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {

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
    void KWJPlannerROS::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        _odom = *odomMsg;
    }

    bool KWJPlannerROS::isValidPose(double x, double y) {
    
    costmap_ros_->updateMap();
    costmap_ = costmap_ros_->getCostmap();

    unsigned int mx, my;
    if (!costmap_->worldToMap(x, y, mx, my))
        return false;
    unsigned char cost = costmap_->getCost(mx, my);
    return (cost == costmap_2d::FREE_SPACE && cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

   double KWJPlannerROS::footprintCost(double x, double y, double th) {
    if (!initialized_) {
        ROS_ERROR("The RRTstarPlanner has not been initialized, you must call initialize().");
        return -1.0;
    }

    costmap_ros_->updateMap();
    costmap_ = costmap_ros_->getCostmap();

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

    if (footprint.size() < 3) return -1.0;

    double footprint_cost = world_model_->footprintCost(x, y, th, footprint);

    return footprint_cost;
   }

    bool KWJPlannerROS::isValidPose(double x, double y, double th) {
       double footprint_cost = footprintCost(x, y, th);

       if ((footprint_cost < 0) || (footprint_cost > 128)) {
            return false;
        }
        return true;
    }


    void KWJPlannerROS::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
    {
      nav_msgs::Path gui_path;
      gui_path.header.frame_id = _odom_frame;
      gui_path.header.stamp = ros::Time::now();
      gui_path.poses = global_plan;
      global_plan_pub_.publish(gui_path);
    }

    void KWJPlannerROS::makeReference(const geometry_msgs::PoseStamped& new_pt_odom,
                                  const geometry_msgs::PoseStamped& current_pose_odom) {
    nav_msgs::Path raw_path;
    raw_path.header.frame_id = _odom_frame;
    raw_path.header.stamp = ros::Time::now();

    double resolution = 0.05;
    double dx = new_pt_odom.pose.position.x - current_pose_odom.pose.position.x;
    double dy = new_pt_odom.pose.position.y - current_pose_odom.pose.position.y;
    double distance = std::hypot(dx, dy);
    int steps = std::max(1, int(distance / resolution));

    for (int i = 0; i <= steps; ++i) {
        double ratio = static_cast<double>(i) / steps;
        geometry_msgs::PoseStamped pt;
        pt.header.frame_id = _odom_frame;
        pt.header.stamp = ros::Time::now();
        pt.pose.position.x = current_pose_odom.pose.position.x + dx * ratio;
        pt.pose.position.y = current_pose_odom.pose.position.y + dy * ratio;
        pt.pose.position.z = 0.0;

        tf2::Quaternion q;
        double yaw = std::atan2(dy, dx);
        q.setRPY(0, 0, yaw);
        pt.pose.orientation = tf2::toMsg(q);

        raw_path.poses.push_back(pt);
    }

    std::vector<geometry_msgs::PoseStamped> smoothed_path = raw_path.poses;
    double obstacle_weight = 0.3;
    double smooth_weight = 1.5;
    double direction_weight = 0.5;
    double max_rep = 0.1;

    for (int iter = 0; iter < 10; ++iter) {
        std::vector<geometry_msgs::PoseStamped> temp_path = smoothed_path;
        for (size_t i = 1; i < smoothed_path.size() - 1; ++i) {
            geometry_msgs::PoseStamped pt = smoothed_path[i];

            double fx = 0.0, fy = 0.0;
            for (const auto& obs : obstacles_) {
                double dx = pt.pose.position.x - obs.first;
                double dy = pt.pose.position.y - obs.second;
                double dist = std::hypot(dx, dy);

                if (dist < 0.5 && dist > 0.01) {
                    double repulsive = obstacle_weight * pow(1.0/dist - 1.0/0.3, 2);
                    fx += repulsive * (dx / dist);
                    fy += repulsive * (dy / dist);
                }

                double rep_mag = hypot(fx, fy);
                if (rep_mag > max_rep) { 
                    fx *= max_rep/rep_mag; 
                    fy *= max_rep/rep_mag; 
                }
        }

            const auto& prev = smoothed_path[i - 1];
            const auto& next = smoothed_path[i + 1];

            double sx = smooth_weight * (prev.pose.position.x + next.pose.position.x - 2 * pt.pose.position.x);
            double sy = smooth_weight * (prev.pose.position.y + next.pose.position.y - 2 * pt.pose.position.y);
            double max_smooth = 0.01;  
            double smooth_step = hypot(sx, sy);

            if (smooth_step > max_smooth) {
               double s = max_smooth / smooth_step;
               sx *= s; sy *= s;
            }             

            double dxg = new_pt_odom.pose.position.x - pt.pose.position.x;
            double dyg = new_pt_odom.pose.position.y - pt.pose.position.y;
            double dist_to_goal = sqrt(dxg * dxg + dyg * dyg);

            dxg /= dist_to_goal;
            dyg /= dist_to_goal;

            double gx = 0.0, gy = 0.0;
            if (dist_to_goal > 0.01 && std::isfinite(dist_to_goal)) {
                gx = direction_weight * dxg;
                gy = direction_weight * dyg;
            }

            double base_x = smoothed_path[i].pose.position.x;
            double base_y = smoothed_path[i].pose.position.y;

            double frac = double(i) / (smoothed_path.size() - 1);
            const auto& anchor = odom_plan[i];
            double k_anchor = 1.5;
            double ax = k_anchor * (anchor.pose.position.x - base_x);
            double ay = k_anchor * (anchor.pose.position.y - base_y);

            double max_step = obstacle_weight*0.03;
            double delta_x = fx + sx + gx + ax;
            double delta_y = fy + sy + gy + ay;
            double step = hypot(delta_x, delta_y);
            if (step > max_step) {
                delta_x = max_step/step;
                delta_y = max_step/step;
            }
            double new_x = base_x + delta_x;
            double new_y = base_y + delta_y;
            double new_th = std::atan2(delta_y, delta_x);

            if (isValidPose(new_x, new_y) && isValidPose(new_x, new_y, new_th)) {
                pt.pose.position.x = new_x;
                pt.pose.position.y = new_y;
                tf2::Quaternion q;
                q.setRPY(0, 0, new_th);
                pt.pose.orientation  = tf2::toMsg(q);
                temp_path[i] = pt;
            }
            
        }
        smoothed_path = temp_path;
    }

        int N = smoothed_path.size();
        Eigen::VectorXd s(N), t(N), xval(N), yval(N);
        s[0] = 0;
        for (int i = 0; i < N; ++i) {
            xval[i] = smoothed_path[i].pose.position.x;
            yval[i] = smoothed_path[i].pose.position.y;
            if (i > 0) {
                double dx = xval[i] - xval[i-1];
                double dy = yval[i] - yval[i-1];
                s[i] = s[i-1] + std::hypot(dx, dy);
            }
        }

        double s_min = 0, s_max = s[N-1];

        for (int i = 0; i < N; ++i) {
            t[i] = 2.0 * (s[i] - s_min) / (s_max - s_min) - 1.0;
        }

        Eigen::VectorXd coeffs_x = polyfit(t, xval, 5);
        Eigen::VectorXd coeffs_y = polyfit(t, yval, 5);

        nav_msgs::Path flex_path;
        flex_path.header.frame_id = _odom_frame;
        flex_path.header.stamp = ros::Time::now();
        double dt = 0.05 * 2.0 / (s_max - s_min);
        for (double t = -1.0; t <= 1.0; t += dt) {
            double x = polyeval(coeffs_x, t);
            double y = polyeval(coeffs_y, t);
            geometry_msgs::PoseStamped p;
            p.header.frame_id = _odom_frame;
            p.header.stamp = ros::Time::now();
            p.pose.position.x = x;
            p.pose.position.y = y;
            flex_path.poses.push_back(p);
        }
        std::vector<geometry_msgs::PoseStamped> flexed_path;
        flexed_path = flex_path.poses;

        nav_msgs::Path final_path;
        final_path.header.frame_id = _odom_frame;
        final_path.header.stamp = ros::Time::now();
        for (const auto& p : flexed_path) {
        if (std::isfinite(p.pose.position.x) && std::isfinite(p.pose.position.y) &&
            std::isfinite(p.pose.orientation.x) && std::isfinite(p.pose.orientation.y) &&
            std::isfinite(p.pose.orientation.z) && std::isfinite(p.pose.orientation.w)) {
            final_path.poses.push_back(p);
            odom_path.poses = final_path.poses;
          }
        }
        _pub_odompath.publish(final_path);
    }

    void KWJPlannerROS::odomPathPublish(const nav_msgs::Path& odom_path){
        nav_msgs::Path gui_path = odom_path;
        gui_path.header.frame_id = _odom_frame;
        gui_path.header.stamp = ros::Time::now(); 
        odom_plan_pub_.publish(gui_path);
    }
    
    void KWJPlannerROS::markPublish(const nav_msgs::Path& path_point){
        if (path_point.poses.empty()) {
            ROS_WARN("markPublish: path_point is empty, skipping marker publish.");
            return;
        }
        
        const auto& ps = path_point.poses.front();
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = _odom_frame;  
        marker.header.stamp = ros::Time::now();
        marker.ns = "odom_arrows";
        marker.id = 0;  
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose = ps.pose; 
        
        marker.scale.x = 0.2;  
        marker.scale.y = 0.05; 
        marker.scale.z = 0.05;
        
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
        
        marker.lifetime = ros::Duration(0);  
        
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_array_pub.publish(marker_array);
    }
        
}
