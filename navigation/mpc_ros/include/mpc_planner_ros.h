#ifndef MPC_LOCAL_PLANNER_NODE_ROS_H
#define MPC_LOCAL_PLANNER_NODE_ROS_H

#include <vector>
#include <map>
#include <Eigen/Core>
// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/simple_trajectory_generator.h>
#include <base_local_planner/simple_scored_sampling_planner.h>
#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
// local planner specific classes which provide some macros
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/server.h>
#include <mpc_ros/MPCPlannerConfig.h>

#include "ros/ros.h"
#include "mpc_planner.h"
#include <iostream>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_listener.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <Eigen/QR>
#include <astar_planner/astar.h>

using namespace std;

namespace mpc_ros{

    class MPCPlannerROS : public nav_core::BaseLocalPlanner
    {
        public:
            MPCPlannerROS();
            ~MPCPlannerROS();
            MPCPlannerROS(std::string name, 
                           tf2_ros::Buffer* tf,
                costmap_2d::Costmap2DROS* costmap_ros);

            // for visualisation, publishers of global and local plan
            ros::Publisher global_plan_pub_;

            void LoadParams(const std::map<string, double> &params);

            // Local planner plugin functions
            void initialize(std::string name, tf2_ros::Buffer* tf,
                costmap_2d::Costmap2DROS* costmap_ros);
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
            void goalSub(geometry_msgs::PoseStamped goal);
            void globalReplanning(const ros::TimerEvent& event);
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            bool mpcComputeVelocityCommands(geometry_msgs::PoseStamped global_pose, geometry_msgs::PoseStamped& global_vel, geometry_msgs::PoseStamped& drive_cmds);
            bool isGoalReached();
            bool isInitialized() {return initialized_;}
            void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
            double getYaw(const geometry_msgs::PoseStamped& pose);
            /**
             * @brief  Update the cost functions before planning
             * @param  global_pose The robot's current pose
             * @param  new_plan The new global plan
             * @param  footprint_spec The robot's footprint
             *
             * The obstacle cost function gets the footprint.
             * The path and goal cost functions get the global_plan
             * The alignment cost functions get a version of the global plan
             *   that is modified based on the global_pose 
             */
            // see constructor body for explanations

        private:
            //Pointer to external objects (do NOT delete object)
            costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap  
            costmap_2d::Costmap2D* costmap_; ///< @brief The costmap the controller will use
            std::string global_frame_; ///< @brief The frame in which the controller will run
            std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot
            std::vector<geometry_msgs::Point> footprint_spec_;
            std::vector<geometry_msgs::PoseStamped> global_plan_,new_global_plan;
      
            base_local_planner::LocalPlannerUtil planner_util_;
            base_local_planner::LatchedStopRotateController latchedStopRotateController_;
            base_local_planner::OdometryHelperRos odom_helper_;
            
            base_local_planner::SimpleTrajectoryGenerator generator_;
            base_local_planner::Trajectory result_traj_;
            base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
            dynamic_reconfigure::Server<MPCPlannerConfig> *dsrv_;
            void reconfigureCB(MPCPlannerConfig &config, uint32_t level);
           
            // Flags
            bool initialized_;
            bool goal_reached_;
            bool rotate;
            bool goal_transformed_;
            bool first;
            bool once;

        private:
            vector<double> mpc_x;
            vector<double> mpc_y;
            vector<double> mpc_theta;

            ros::NodeHandle _nh;
            ros::Subscriber _sub_odom;
            ros::Publisher _pub_odompath, _pub_mpctraj;
            ros::Publisher global_astar_pub_;
            geometry_msgs::PoseStamped goal_,start;
            ros::Subscriber goal_sub_;
            ros::Timer global_timer_;
            tf2_ros::Buffer *tf_;  
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf_listener_;
            
            nav_msgs::Odometry _odom;
            nav_msgs::Path _odom_path, _mpc_traj; 
            geometry_msgs::Twist _twist_msg;
            geometry_msgs::PoseStamped current_pose_;

            string _map_frame, _odom_frame, _base_frame;

            MPC _mpc;
            map<string, double> _mpc_params;
            double _mpc_steps, _ref_cte, _ref_etheta, _ref_vel, _w_cte, _w_etheta, _w_vel, 
                _w_angvel, _w_accel, _w_angvel_d, _w_accel_d, _max_angvel, _max_throttle, _bound_value;

            double _dt, _w, _throttle, _speed, _max_speed;
            double _pathLength, _goalRadius, _waypointsDist;
            double xy_goal_tolerance_;
            double yaw_error,del_x;
            std::vector<unsigned int> path;
            int _downSampling;
            int size_x_; 
            bool _debug_info, _delay_mode;
            double polyeval(Eigen::VectorXd coeffs, double x);
            Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

            unsigned int start_x,start_y,goal_x,goal_y;
            astar_planner::AStarPlanner astar;

            void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
            void desiredPathCB(const nav_msgs::Path::ConstPtr& pathMsg);
            void controlLoopCB(const ros::TimerEvent&);
            
    };
};
#endif /* MPC_LOCAL_PLANNER_NODE_ROS_H */
