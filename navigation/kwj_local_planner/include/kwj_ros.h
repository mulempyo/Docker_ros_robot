#ifndef KWJ_ROS_H
#define KWJ_ROS_H

#include <vector>
#include <map>
#include <Eigen/Core>
#include <angles/angles.h>

#include <nav_core/base_local_planner.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <tf/tf.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <dynamic_reconfigure/server.h>
#include <kwj_local_planner/KWJPlannerConfig.h>

#include "ros/ros.h"
#include "kwj.h"
#include <iostream>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_listener.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <Eigen/QR>

using namespace std;

namespace kwj_local_planner{

    class KWJPlannerROS : public nav_core::BaseLocalPlanner
    {
        public:
            KWJPlannerROS();
            ~KWJPlannerROS();
            KWJPlannerROS(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

            ros::Publisher global_plan_pub_, odom_plan_pub_;

            void LoadParams(const std::map<string, double> &params);

            // Local planner plugin functions
            void initialize(std::string name, tf2_ros::Buffer* tf,
                costmap_2d::Costmap2DROS* costmap_ros);
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            bool kwjComputeVelocityCommands(geometry_msgs::PoseStamped global_pose, geometry_msgs::PoseStamped& global_vel, geometry_msgs::PoseStamped& drive_cmds);
            bool isGoalReached();
            void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
            void odomPathPublish(const nav_msgs::Path& path_point);
            void markPublish(const nav_msgs::Path& odom_path);

        private:
            costmap_2d::Costmap2DROS* costmap_ros_; 
            costmap_2d::Costmap2D* costmap_; 
            std::string global_frame_; 
            std::string robot_base_frame_; 
            std::vector<geometry_msgs::Point> footprint_spec_;
            std::vector<geometry_msgs::PoseStamped> global_plan_;
            std::vector<std::pair<double,double>> obstacles_;
      
            base_local_planner::LocalPlannerUtil planner_util_;
            base_local_planner::OdometryHelperRos odom_helper_;
            
            base_local_planner::Trajectory result_traj_;
            dynamic_reconfigure::Server<KWJPlannerConfig> *dsrv_;
            void reconfigureCB(KWJPlannerConfig &config, uint32_t level);
           
            bool initialized_;
            bool goal_reached_;
         

        private:
            vector<double> kwj_x;
            vector<double> kwj_y;
            vector<double> kwj_theta;

            ros::NodeHandle _nh;
            ros::Subscriber _sub_odom;
            ros::Publisher _pub_odompath, _pub_kwjtraj, marker_array_pub;
           
            tf2_ros::Buffer *tf_;  
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf_listener_;
            
            nav_msgs::Odometry _odom;
            nav_msgs::Path _odom_path, _kwj_traj; 
            geometry_msgs::Twist _twist_msg;
            geometry_msgs::PoseStamped current_pose_;

            string _map_frame, _odom_frame, _base_frame;

            KWJ _kwj;
            map<string, double> _kwj_params;
            double _kwj_steps, _ref_cte, _ref_etheta, _ref_vel, _w_cte, _w_etheta, _w_vel, 
                _w_angvel, _w_accel, _w_angvel_d, _w_accel_d, _max_angvel, _max_a, _bound_value, _t_max, _dt_min, _dt_max, _o_min, _o_weight;

            double _dt, _w, _a, _speed, _max_speed;
            double xy_goal_tolerance_;
            double orientation_tolerance_;
            double yaw_error;
            double resolution;           
            double origin_x;              
            double origin_y;
            double width,height;
            double inflation_radius;
            std::vector<unsigned int> path;
            int size_x_; 
            bool _debug_info, _delay_mode;

            double polyeval(Eigen::VectorXd coeffs, double x);
            Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
            void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        
            
    };
};
#endif /* KWJ_ROS_H */
