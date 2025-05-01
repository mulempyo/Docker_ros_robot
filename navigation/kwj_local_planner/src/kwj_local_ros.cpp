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

	KWJPlannerROS::~KWJPlannerROS() {}

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
    _pub_kwjtraj = private_nh.advertise<nav_msgs::Path>("kwj_trajectory",1);
    _pub_odompath = private_nh.advertise<nav_msgs::Path>("kwj_reference",1);

    std::string controller_frequency_param_name;
    double controller_frequency = 0;
        if(!private_nh.searchParam("move_base/controller_frequency", controller_frequency_param_name)) {
            ROS_WARN("controller_frequency_param_name doesn't exits");
        } else {
            private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
            
            if(controller_frequency > 0) {
            } else {
                ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
            }
        }
    _dt = double(1.0/controller_frequency);

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
   
        nav_msgs::Path odom_path;
        odom_path.poses.clear();

        std::vector<geometry_msgs::PoseStamped> sampled;
        double interval = 0.5;      
        double dist_accum = 0.0;
        double angle_to_goal;

        geometry_msgs::PoseStamped current_pose;
        if (!costmap_ros_->getRobotPose(current_pose)) {
          ROS_ERROR("Could not get robot pose");
          return false;
        }

        double robot_pose_x = current_pose.pose.position.x;
        double robot_pose_y = current_pose.pose.position.y;
        double robot_pose_theta = tf2::getYaw(current_pose.pose.orientation);

        _kwj.currentPose(robot_pose_x, robot_pose_y, robot_pose_theta);

        if (!global_plan_.empty()) {
          sampled.push_back(global_plan_.front());

          for (size_t i = 1; i < global_plan_.size(); ++i) {
            const auto &prev = global_plan_[i - 1];
            const auto &curr = global_plan_[i];

            double dx = curr.pose.position.x - prev.pose.position.x;
            double dy = curr.pose.position.y - prev.pose.position.y;
            double seg_len = std::hypot(dx, dy);
            if (seg_len < 1e-6) continue;

             double remaining = seg_len;
             geometry_msgs::PoseStamped last_pt = prev;

             while (dist_accum + remaining >= interval) {
               double d = interval - dist_accum; 
               double ratio = d / remaining;         

               geometry_msgs::PoseStamped new_pt;
               new_pt.header = curr.header;          
               new_pt.pose.position.x = last_pt.pose.position.x + ratio * dx;
               new_pt.pose.position.y = last_pt.pose.position.y + ratio * dy;
 
               double yaw = std::atan2(dy, dx);
               new_pt.pose.orientation = tf2::toMsg(tf2::Quaternion(
               tf2::Vector3(0,0,1), yaw));

               sampled.push_back(new_pt);

               last_pt = new_pt;
               remaining -= d;
               dist_accum = 0.0;

               dx = curr.pose.position.x - last_pt.pose.position.x;
               dy = curr.pose.position.y - last_pt.pose.position.y;

               double del_x = new_pt.pose.position.x - robot_pose_x;
               double del_y = new_pt.pose.position.y - robot_pose_y;
               angle_to_goal = atan2(del_y, del_x);

               tf2::Quaternion q;
               q.setRPY(0, 0, angle_to_goal);
               new_pt.pose.orientation = tf2::toMsg(q);

               double robot_yaw = robot_pose_theta;
               double target_yaw = atan2(new_pt.pose.position.y - robot_pose_y, new_pt.pose.position.x - robot_pose_x); 

               yaw_error = angles::shortest_angular_distance(robot_yaw, target_yaw);            
              }

               dist_accum += remaining;
            }

          const auto &final = global_plan_.back();
          const auto &last_samp = sampled.back();
          double dx_end = final.pose.position.x - last_samp.pose.position.x;
          double dy_end = final.pose.position.y - last_samp.pose.position.y;
          sampled.push_back(final);
          
        }
    
        for (auto &gp : sampled) {
        try {
            geometry_msgs::PoseStamped odom_pose =
            tf_buffer_.transform(gp, _odom_frame, ros::Duration(1.0));
            odom_path.poses.push_back(odom_pose);
          }
        catch (tf2::TransformException &ex) {
            ROS_ERROR("transform error: %s", ex.what());
            return false;
          }
        }
        
        if (!odom_path.poses.empty()) {
           odom_path.header.frame_id = _odom_frame;
           odom_path.header.stamp    = ros::Time::now();
           _pub_odompath.publish(odom_path);
        } else {
           ROS_DEBUG_NAMED("kwj_ros", "Odom path is empty after sampling.");
          return false;
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
       
        vector<double> kwj_results = _kwj.Solve(state, coeffs);    
        
        _w = kwj_results[0]; // radian/sec, angular velocity
        _a = kwj_results[1]; // acceleration
          
        
        _speed = v + _a * dt;  // speed
        if (_speed >= _max_speed)
            _speed = _max_speed;
        if(_speed <= 0.0)
            _speed = 0.0;

        _kwj_traj = nav_msgs::Path();
        _kwj_traj.header.frame_id = _base_frame;      
        _kwj_traj.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped tempPose;
        tf2::Quaternion myQuaternion;
        for(int i=0; i<_kwj.kwj_x.size(); i++)
        {
            tempPose.header = _kwj_traj.header;
            tempPose.pose.position.x = _kwj.kwj_x[i];
            tempPose.pose.position.y = _kwj.kwj_y[i];
            myQuaternion.setRPY( 0, 0, _kwj.kwj_theta[i] );  
            tempPose.pose.orientation.x = myQuaternion[0];
            tempPose.pose.orientation.y = myQuaternion[1];
            tempPose.pose.orientation.z = myQuaternion[2];
            tempPose.pose.orientation.w = myQuaternion[3];
                
            _kwj_traj.poses.push_back(tempPose); 
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
        _pub_kwjtraj.publish(_kwj_traj);

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


    void KWJPlannerROS::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
    {
      nav_msgs::Path gui_path;
      gui_path.header.frame_id = _map_frame;
      gui_path.header.stamp = ros::Time::now();
      gui_path.poses = global_plan;
      global_plan_pub_.publish(gui_path);
    }
}
