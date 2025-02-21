#ifndef DWA_PLANNER_ROS_H
#define DWA_PLANNER_ROS_H

#include <ros/ros.h>
#include <tf2/utils.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/local_planner_util.h>
#include "dwa_planner_ros/dwa_planner.h"
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <torch/torch.h>
#include <mutex>
#include <astar_planner/astar.h>

namespace dwa_planner_ros{

  class FuzzyNN : public torch::nn::Module {
    public:
        FuzzyNN() {
            
            fc1 = register_module("fc1", torch::nn::Linear(3, 6));
            fc2 = register_module("fc2", torch::nn::Linear(6, 9));
            activation = register_module("activation", torch::nn::Functional(torch::relu));
        }
    
        torch::Tensor forward(torch::Tensor x) {
        x = torch::relu(fc1->forward(x));
        x = fc2->forward(x);
        x = torch::sigmoid(x);  
        return x;
      }
        
    private:
        torch::nn::Linear fc1{nullptr}, fc2{nullptr};
        torch::nn::Functional activation{torch::relu};
    };  

class DWAPlannerROS: public nav_core::BaseLocalPlanner {
public:

  DWAPlannerROS();
  ~DWAPlannerROS();

  /**
   * @brief Initializes the DWAPlannerROS.
   *
   * @param name The name of the planner.
   * @param tf A pointer to the tf2_ros buffer.
   * @param costmap_ros A pointer to the costmap_2d::Costmap2DROS.
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
  void safeMode(std_msgs::Float64 safe);
  void personDetect(const std_msgs::Float64::ConstPtr& person);
  void odomSub(const nav_msgs::Odometry odomMsg);
  void goalSub(geometry_msgs::PoseStamped goal);
  void scanCallback(const sensor_msgs::LaserScan& scan);
  void trainCallback(const ros::TimerEvent& event);
  void replanning(const ros::TimerEvent& event);
  void globalReplanning(const ros::TimerEvent& event);

  /**
   * @brief Set the global plan for the local planner.
   *
   * @param orig_global_plan The global plan from the global planner.
   * @return True if successful, false otherwise.
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief Compute velocity commands for the robot to follow the global plan.
   *
   * @param cmd_vel The output velocity command for the robot.
   * @return True if a valid command is found, false otherwise.
   */
  
  double triangular_mf(double x, double a, double b, double c);
  std::vector<double> fuzzify_distance(double distance);
  std::vector<double> fuzzify_speed(double speed);
  std::vector<double> fuzzify_angular_velocity(double omega);
  void updateTrainingData(double distance_);

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief Check if the robot has reached its goal.
   *
   * @return True if the goal is reached, false otherwise.
   */
  bool isGoalReached();

  double computeRepulsiveForce(double distance, double min_potential_distance);

  std::vector<geometry_msgs::PoseStamped> global_plan_,new_global_plan;
  ros::Publisher safe_pub_;
  ros::Publisher global_astar_pub_;
  ros::Subscriber sub_;
  ros::Subscriber person_sub_;
  ros::Subscriber amcl_sub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber goal_sub_;
  ros::Timer train_timer_;
  ros::Timer path_timer_;
  ros::Timer global_timer_;

private:

  void trainFuzzyNN();
  /**
   * @brief Allocates memory for the costmap.
   */
  void allocateMemory();

  /**
   * @brief Frees the memory allocated for the costmap.
   */
  void freeMemory();

  /**
   * @brief Publishes the global plan for visualization.
   *
   * @param global_plan The global plan to publish.
   */
  void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);

  double getYaw(const geometry_msgs::PoseStamped& pose);

  bool initialized_;            ///< Whether the planner is initialized or not.
  bool rotate;
  bool goal_reached_;           ///< Whether the goal is reached or not.
  bool person_detect;
  bool safe_mode;
  bool success;
  bool first;
  bool goal_transformed_;

  int size_x_;                  ///< Size of the costmap in the x direction.
  int size_y_;                  ///< Size of the costmap in the y direction.
  int size_x;
  int size_y;
  unsigned char** charmap_ = nullptr;      ///< The costmap data.
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_; ///< Pointer to the costmap.
  base_local_planner::CostmapModel* costmap_model_ = nullptr;  ///< Costmap model used for collision checking.
  DWAPlanner* planner_ = nullptr;         ///< The DWA planner instance.
  ros::Publisher global_plan_pub_;  ///< Publisher for the global plan.
  tf2_ros::Buffer* tf_;
  // Parameters   
  std::string odom_topic_;
  std::string map_frame_;       ///< Map frame name.
  std::string global_frame_;
  std::string robot_base_frame_;
  double xy_goal_tolerance_;    ///< Goal tolerance in the x and y directions.
  double transform_tolerance_;  ///< Tolerance for transforming coordinates.
  double max_vel_x_;            ///< Maximum linear velocity.
  double min_vel_x_;            ///< Minimum linear velocity.
  double max_vel_theta_;        ///< Maximum angular velocity.
  double min_vel_theta_;        ///< Minimum angular velocity.
  double acc_lim_x_;            ///< Maximum linear acceleration.
  double acc_lim_theta_;        ///< Maximum angular acceleration.
  double control_period_;       ///< Control loop period.
  double repulsive_factor_;
  double min_potential_distance_;

  double resolution;            ///< Resolution of the costmap.
  double origin_x;              ///< Origin x coordinate of the costmap.
  double origin_y;              ///< Origin y coordinate of the costmap.
  std::vector<geometry_msgs::Point> footprint_spec_;  ///< Footprint of the robot.
  double robot_inscribed_radius_;  ///< The inscribed radius of the robot.
  double robot_circumscribed_radius_;  ///< The circumscribed radius of the robot.
  double distance_;
  double accuracy;
  double robot_vel_x;
  double robot_vel_theta;
  double robot_pose_x;
  double robot_pose_y;
  double robot_pose_theta;
  double dwa_cmd_vel_x, dwa_cmd_vel_theta;
  unsigned int start_x,start_y,goal_x,goal_y;
  geometry_msgs::PoseStamped goal_;
  geometry_msgs::PoseStamped start_;

  std::vector<unsigned int> path;
  std::vector<std::array<float, 7>> safes;
  
  geometry_msgs::PoseStamped current_pose_;  ///< The current pose of the robot.

  base_local_planner::OdometryHelperRos odom_helper_;  ///< Helper to get odometry data.
  base_local_planner::LocalPlannerUtil planner_util_;       ///< Utility to assist with planning.
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::mutex train_mutex_;
  torch::Tensor input_;
  nav_msgs::Odometry _odom;
  ros::NodeHandle nh_;
  astar_planner::AStarPlanner astar;

  torch::nn::Sequential fuzzy_nn;
  std::vector<std::vector<double>> train_inputs;
  std::vector<std::vector<double>> train_outputs;
  std::vector<std::vector<double>> reference_path;
  std::vector<double> dis_vector, vel_x_vector, vel_theta_vector;

};

}  // namespace dwa

#endif  // DWA_PLANNER_ROS_H
