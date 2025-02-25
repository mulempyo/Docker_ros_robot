#include "dwa_planner_ros/dwa_planner.h"
#include <Eigen/Core>
#include <cmath>
#include <angles/angles.h>

namespace dwa_planner_ros {

DWAPlanner::DWAPlanner(base_local_planner::CostmapModel* costmap_model,
                       const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius,
                       double circumscribed_radius, ros::NodeHandle& nh)
  : costmap_model_(costmap_model)
  , footprint_spec_(footprint_spec)
  , inscribed_radius_(inscribed_radius)
  , circumscribed_radius_(circumscribed_radius)
{
  candidate_paths_pub_ = nh.advertise<nav_msgs::Path>("dwa_candidate_paths", 1);
  nh.param("map_frame", map_frame_, std::string("map"));
  nh.param("max_vel_x", max_vel_x_, 0.35);
  nh.param("min_vel_x", min_vel_x_, 0.0);
  nh.param("max_vel_theta", max_vel_theta_, 2.5);
  nh.param("min_vel_theta", min_vel_theta_, -2.5);
  nh.param("acc_lim_x", acc_lim_x_, 0.15);
  nh.param("acc_lim_theta", acc_lim_theta_, 1.5);
  nh.param("control_period", control_period_, 0.2);
  nh.param("path_distance_bias", path_distance_bias_, 3.0);
  nh.param("goal_distance_bias", goal_distance_bias_, 2.0);
  nh.param("occdist_scale", occdist_scale_, 0.01);
  nh.param("sim_time_samples", sim_time_samples_, 10);
  nh.param("vx_samples", vx_samples_, 10);
  nh.param("vth_samples", vth_samples_, 20);
  nh.param("path_follow", path_follow_, 15.0);
}

DWAPlanner::~DWAPlanner() {}

bool DWAPlanner::computeVelocityCommands(const double& robot_vel_x, const double& robot_vel_theta,
  const double& robot_pose_x, const double& robot_pose_y, const double& robot_pose_theta,
  const std::vector<std::vector<double>>& global_plan, unsigned char const* const* costmap,
  int size_x, int size_y, double resolution, double origin_x, double origin_y,
  std::vector<double> dis_vector, std::vector<double> vel_x_vector, std::vector<double> vel_theta_vector,
  double& cmd_vel_x, double& cmd_vel_theta)
{
size_x_ = size_x;
size_y_ = size_y;

std::vector<std::pair<double, double>> sample_vels;
if (!samplePotentialVels(robot_vel_x, robot_vel_theta, sample_vels, dis_vector, vel_x_vector, vel_theta_vector)){
return false;
}

std::vector<std::vector<double>> pruned_global_plan = cutGlobalPlan(global_plan, size_x, size_y, robot_pose_x, robot_pose_y);
std::vector<std::pair<double, double>>::iterator it = sample_vels.begin();
std::vector<std::vector<std::vector<double>>> path_all;
double min_score = 1e10;

while (it != sample_vels.end()) {
std::vector<std::vector<double>> traj;
generateTrajectory(robot_vel_x, robot_vel_theta, robot_pose_x, robot_pose_y, robot_pose_theta, it->first, it->second, traj, dis_vector,vel_x_vector, vel_theta_vector, global_plan);

double score = scoreTrajectory(traj, size_x, size_y, resolution, origin_x, origin_y, pruned_global_plan, costmap);

double yaw_correction = std::max(0.0, 1.0 - std::abs(yaw_error_)); 
double angular_correction = yaw_error_; 

double adjusted_vel_x = std::max(min_vel_x_, std::min(it->first * yaw_correction, max_vel_x_));
double adjusted_vel_theta = std::max(min_vel_theta_, std::min(it->second + angular_correction, max_vel_theta_));

if (score < min_score) {
min_score = score;
cmd_vel_x = adjusted_vel_x;
cmd_vel_theta = adjusted_vel_theta;
}

path_all.push_back(traj);
++it;
}

publishCandidatePaths(path_all);
return true;
}


std::vector<std::vector<double>> DWAPlanner::cutGlobalPlan(const std::vector<std::vector<double>>& global_plan,
  const int& size_x, const int& size_y, const double& robot_pose_x, const double& robot_pose_y)
{
  std::vector<std::vector<double>> pruned_global_plan;
  for (auto it = global_plan.begin(); it != global_plan.end(); ++it) {
  if (std::abs(it->at(0) - robot_pose_x) > size_x / 2.0 && std::abs(it->at(1) - robot_pose_y) > size_y / 2.0) {
    break;
  }
  pruned_global_plan.push_back(*it);
  }
  return pruned_global_plan;
}

double DWAPlanner::scoreTrajectory(const std::vector<std::vector<double>>& traj, const int& size_x, const int& size_y,
  const double& resolution, const double& origin_x, const double& origin_y,
  const std::vector<std::vector<double>>& global_plan, unsigned char const* const* costmap)
{
int mx, my;
int occupy = 0;
for (auto it = traj.begin(); it != traj.end(); ++it) {
worldToMap(it->at(0), it->at(1), mx, my, resolution, origin_x, origin_y);
if(mx >= 0 && my >= 0 && mx < size_x && my < size_y){
occupy = std::max(costmap[mx][my] - 0, occupy);
}
}

const std::vector<double>& end_pose = traj.back();
const std::vector<double>& local_end_pose = global_plan.back();
double dis2end = std::sqrt(std::pow(end_pose[0] - local_end_pose[0], 2) + std::pow(end_pose[1] - local_end_pose[1], 2));

double dis2path = 1e10;
for (auto it = global_plan.begin(); it != global_plan.end(); ++it) {
dis2path = std::min(std::sqrt(std::pow(end_pose[0] - it->at(0), 2) + std::pow(end_pose[1] - it->at(1), 2)), dis2path);
}

double follow = isTrajectoryAdherent(traj, global_plan);

return occdist_scale_ * occupy + goal_distance_bias_ * dis2end + path_distance_bias_ * dis2path + path_follow_ * follow;
}



void DWAPlanner::generateTrajectory(const double& robot_vel_x, const double& robot_vel_theta,
                                    const double& robot_pose_x, const double& robot_pose_y, const double& robot_pose_theta,
                                    const double& sample_vel_x, const double& sample_vel_theta, std::vector<std::vector<double>>& traj,
                                    std::vector<double> dis_vector, std::vector<double> vel_x_vector, std::vector<double> vel_theta_vector, const std::vector<std::vector<double>>& global_plan)
{
  double pose_x = robot_pose_x;
  double pose_y = robot_pose_y;
  double pose_theta = robot_pose_theta;
  double vel_x = robot_vel_x;
  double vel_theta = robot_vel_theta;

  for (auto i = 0; i < sim_time_samples_; ++i) {
    vel_x = computeNewLinearVelocities(vel_x_vector, dis_vector, sample_vel_x, vel_x, acc_lim_x_);
    vel_theta = computeNewAngularVelocities(vel_theta_vector, dis_vector, sample_vel_theta, vel_theta, acc_lim_theta_);
    computeNewPose(pose_x, pose_y, pose_theta, vel_x, vel_theta, global_plan);
    traj.push_back({pose_x, pose_y, pose_theta});
  }
}

void DWAPlanner::worldToMap(const double wx, const double wy, int& mx, int& my, const double resolution, const double origin_x, const double origin_y){
  mx = int((wx-origin_x)/resolution);
  my = int((wy-origin_y)/resolution);
}

void DWAPlanner::computeNewPose(double& pose_x, double& pose_y, double& pose_theta,
  const double& vel_x, const double& vel_theta, const std::vector<std::vector<double>>& global_plan)
{
    double x, y, th;
    x = vel_x * std::cos(pose_theta + yaw_error_) * control_period_;
    y = vel_x * std::sin(pose_theta + yaw_error_) * control_period_;
    th = vel_theta * control_period_;
    if(isValidPose(x, y, th)){
      pose_x += x;
      pose_y += y;
      pose_theta += th;
   }
}


double DWAPlanner::computeNewLinearVelocities(std::vector<double> vel_x_vector, std::vector<double> dis_vector, const double& target_vel, double& current_vel, const double& acc_lim)
{
  if (target_vel < current_vel) {
    return std::max(target_vel, current_vel - acc_lim_x_ * control_period_);
  } else {
    return std::min(target_vel, current_vel + acc_lim_x_ * control_period_);
  }

}

double DWAPlanner::computeNewAngularVelocities(std::vector<double> vel_theta_vector, std::vector<double> dis_vector, const double& target_vel, double& current_vel, const double& acc_lim)
{      
    if (obstacleDetected()) {
        if(target_vel < current_vel){
          return std::max(target_vel, current_vel - acc_lim_theta_ * control_period_);
        }else{
          return std::min(target_vel, current_vel + acc_lim_theta_ * control_period_);
        }
    } else{
        if(target_vel < current_vel){
          return std::max(target_vel, current_vel - acc_lim_theta_ * control_period_);
        }else{
          return std::min(target_vel, current_vel + acc_lim_theta_ * control_period_);
        }
    }
}

bool DWAPlanner::samplePotentialVels(const double& robot_vel_x, const double& robot_vel_theta,
                                     std::vector<std::pair<double, double>>& sample_vels, std::vector<double> dis_vector, std::vector<double> vel_x_vector, std::vector<double> vel_theta_vector)
{
  double min_vel_x = std::max(min_vel_x_, robot_vel_x - acc_lim_x_ * control_period_);
  double max_vel_x = std::min(max_vel_x_, robot_vel_x + acc_lim_x_ * control_period_);
  double min_vel_theta = std::max(min_vel_theta_, robot_vel_theta - acc_lim_theta_ * control_period_);
  double max_vel_theta = std::min(max_vel_theta_, robot_vel_theta + acc_lim_theta_ * control_period_);
  
  for (double v = min_vel_x; v <= max_vel_x; v += (max_vel_x - min_vel_x) / vx_samples_) {
    for (double w = min_vel_theta; w <= max_vel_theta; w += (max_vel_theta - min_vel_theta) / vth_samples_) {
      sample_vels.push_back(std::make_pair(v, w));
    }
  }

  return !sample_vels.empty();
}

bool DWAPlanner::isPathFeasible(const std::vector<std::vector<double>>& path)
{
  int look_ahead_idx = path.size() - 1;

  for (int i = 0; i <= look_ahead_idx; ++i) {
    if (costmap_model_->footprintCost(path[i][0], path[i][1], path[i][2], footprint_spec_, inscribed_radius_, circumscribed_radius_) == -1) {
      return false;
    }
  }

  return true;
}

void DWAPlanner::publishCandidatePaths(const std::vector<std::vector<std::vector<double>>>& candidate_paths)
{
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = map_frame_;
  gui_path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;

  for (int i = 0; i < (int)candidate_paths.size(); i++)
  {
    for (int j = 0; j < (int)candidate_paths[i].size(); j++)
    {
      double x = candidate_paths[i][j][0];  // x position
      double y = candidate_paths[i][j][1];  // y position
      double theta = candidate_paths[i][j][2];  // orientation (theta)
      
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
      gui_path.poses.push_back(pose);
    }

    for (int j = (int)candidate_paths[i].size() - 1; j >= 0; j--)
    {
      double x = candidate_paths[i][j][0];  // x position
      double y = candidate_paths[i][j][1];  // y position
      double theta = candidate_paths[i][j][2];  // orientation (theta)

      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
      gui_path.poses.push_back(pose);
    }
  }

  candidate_paths_pub_.publish(gui_path);
}

std::vector<std::pair<int, int>> DWAPlanner::bresenhamLine(int x0, int y0, int x1, int y1) {
        std::vector<std::pair<int, int>> points;
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2, e2;

        while (true) {
            points.push_back(std::make_pair(x0, y0));
            if (x0 == x1 && y0 == y1) break;
            e2 = err;
            if (e2 > -dx) { err -= dy; x0 += sx; }
            if (e2 < dy) { err += dx; y0 += sy; }
        }
        return points;
    } 

    double DWAPlanner::isTrajectoryAdherent(const std::vector<std::vector<double>>& traj,
      const std::vector<std::vector<double>>& global_plan)
    {
       double total_error = 0.0;
    
       for (const auto& pt : traj) {
         double min_dist = std::numeric_limits<double>::max();
         double desired_theta = 0.0;
       for (const auto& gp : global_plan) {
         double dx = pt[0] - gp[0];
         double dy = pt[1] - gp[1];
         double d = std::hypot(dx, dy);
       if (d == 0.15) {
         min_dist = d;
         desired_theta = gp[2];  
        }
        }
       double error = fabs(angles::shortest_angular_distance(pt[2], desired_theta));
       total_error += error;
       }
       return total_error;
    }    

bool DWAPlanner::isValidPose(double x, double y, double th) const {
 
  std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
  double cost = costmap_model_->footprintCost(x, y, th, footprint);
  if(cost < 0 || cost >= costmap_2d::LETHAL_OBSTACLE){
    return false;
  }else{
    return true;
  }
}


void DWAPlanner::costmap(costmap_2d::Costmap2D* costmap, costmap_2d::Costmap2DROS* costmap_ros){
  costmap_ = costmap;
  costmap_ros_ = costmap_ros;
}

void DWAPlanner::obstacleFound(bool found){
     obstacle_found = found;
}

bool DWAPlanner::obstacleDetected() const {
    return obstacle_found;
}

void DWAPlanner::yaw(double yaw_error){
  yaw_error_ = yaw_error;
}


}
