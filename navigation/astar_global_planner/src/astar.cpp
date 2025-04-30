#include <astar_planner/astar.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <mutex>
#include <thread>
#include <tf/tf.h>
#include <cuda_runtime.h>

// Register the A* planner as a plugin
PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, nav_core::BaseGlobalPlanner)

extern "C" void cudaAStar(
    unsigned int current_x,
    unsigned int current_y,
    unsigned int current_index,
    int num_points,                     
    const unsigned char* costmap,
    double resolution,
    int width,
    int height,
    double* g_cost,
    unsigned int* came_from,
    int goal_x,
    int goal_y,
    double* open_list_costs,
    unsigned int* open_list_indices,
    int* open_list_size,
    int* h_open_list_size);

namespace astar_planner {

    AStarPlanner::AStarPlanner() : costmap_(nullptr), initialized_(false) {}

    AStarPlanner::AStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
        : costmap_(nullptr), initialized_(false) {
        ros::NodeHandle nh;
        nh_ = nh;
        initialize(name, costmap_ros);
    }

    AStarPlanner::~AStarPlanner() {}

    void AStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        if (!initialized_) {
            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("astar_plan", 1);
            goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("/goal", 1);
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            origin_x_ = costmap_->getOriginX();
            origin_y_ = costmap_->getOriginY();
            resolution_ = costmap_->getResolution();
            width_ = costmap_->getSizeInCellsX();
            height_ = costmap_->getSizeInCellsY();
            global_frame_ = costmap_ros_->getGlobalFrameID();
            costmap_model_ = new base_local_planner::CostmapModel(*costmap_);
            initialized_ = true;
        } else {
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }
    }

    bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                                const geometry_msgs::PoseStamped& goal, 
                                std::vector<geometry_msgs::PoseStamped>& plan) {
        boost::mutex::scoped_lock lock(mutex_); 

        if (!initialized_) {
            ROS_ERROR("AStarPlanner has not been initialized, please call initialize() before use.");
            return false;
        }

        plan.clear();
        
        goal_pub_.publish(goal);

        double wx = start.pose.position.x;
        double wy = start.pose.position.y;

        if (!costmap_->worldToMap(wx, wy, start_x, start_y)) {
            ROS_WARN("The start is out of the map bounds.");
            return false;
        }

        wx = goal.pose.position.x;
        wy = goal.pose.position.y;

        if (!costmap_->worldToMap(wx, wy, goal_x, goal_y)) {
            ROS_WARN("The goal is out of the map bounds.");
            return false;
        }

        path = aStarSearch(start_x, start_y, goal_x, goal_y);

        if (path.empty()) {
            ROS_WARN("Failed to find a valid plan.");
            return false;
        }

        if(!path.empty()){
          path.erase(path.begin());
        }

        std::vector<double> x_vals, y_vals;
        for (unsigned int index : path) {
            unsigned int mx = index % width_;  
            unsigned int my = index / width_;
            double wx, wy;
            mapToWorld(mx, my, wx, wy);  
            x_vals.push_back(wx);
            y_vals.push_back(wy);
        }

        Eigen::VectorXd X = Eigen::Map<Eigen::VectorXd>(x_vals.data(), x_vals.size());
        Eigen::VectorXd Y = Eigen::Map<Eigen::VectorXd>(y_vals.data(), y_vals.size());

        int poly_order = 3;
        Eigen::VectorXd coeffs = polyfit(X, Y, poly_order);

        double ds = 0.1;

        std::vector<geometry_msgs::PoseStamped> smooth_plan;
        geometry_msgs::PoseStamped first_pose;
        first_pose.header.frame_id = "map"; 
        first_pose.header.stamp = ros::Time::now();
        first_pose.pose.position.x = x_vals.front();
        first_pose.pose.position.y = y_vals.front();
        first_pose.pose.position.z = 0;
        smooth_plan.push_back(first_pose);

        double x_start = x_vals.front();
        double x_end = x_vals.back();
        double prev_x = x_start, prev_y = y_vals.front();


        for (double x = x_start; x < x_end; x += ds) {
           double y = polyval(coeffs, x);
           geometry_msgs::PoseStamped pose;
           pose.header.frame_id = "map";
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
         last_pose.header.frame_id = "map";
         last_pose.header.stamp = ros::Time::now();
         last_pose.pose.position.x = x_vals.back();
         last_pose.pose.position.y = y_vals.back();
         last_pose.pose.position.z = 0;
         smooth_plan.push_back(last_pose);


        for (const auto &pose : smooth_plan) {
          plan.push_back(pose);
        }
        publishPlan(plan);
        return true;
        }
   
    void AStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (!initialized_) {
            ROS_ERROR("AStarPlanner has not been initialized, please call initialize() before use.");
            return;
        }
 
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());
        
        if (path.empty()) {
            gui_path.header.frame_id = global_frame_;
            gui_path.header.stamp = ros::Time::now();
        } else { 
            gui_path.header.frame_id = path[0].header.frame_id;
            gui_path.header.stamp = path[0].header.stamp;
        }

        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }

    Eigen::VectorXd AStarPlanner::polyfit(const Eigen::VectorXd& x, const Eigen::VectorXd& y, int order) {
        Eigen::MatrixXd A(x.size(), order + 1);
        for (int i = 0; i < x.size(); i++) {
          for (int j = 0; j < order + 1; j++) {
            A(i, j) = std::pow(x(i), j);
          }
        }
        Eigen::VectorXd coeffs = A.householderQr().solve(y);
        return coeffs;
      }
      
      double AStarPlanner::polyval(const Eigen::VectorXd& coeffs, double x) {
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++) {
          result += coeffs(i) * std::pow(x, i);
        }
        return result;
      }

    std::vector<unsigned int> AStarPlanner::aStarSearch(unsigned int start_x, unsigned int start_y, unsigned int goal_x, unsigned int goal_y) {
        unsigned int start_index = start_y * width_ + start_x;
        unsigned int goal_index = goal_y * width_ + goal_x;

        std::vector<double> g_cost(width_ * height_, std::numeric_limits<double>::infinity());
        std::vector<unsigned int> came_from(width_ * height_, 0);
        std::priority_queue<std::pair<double, unsigned int>, std::vector<std::pair<double, unsigned int>>, std::greater<std::pair<double, unsigned int>>> open_list;

        g_cost[start_index] = 0.0;
        open_list.emplace(heuristic(start_x, start_y, goal_x, goal_y), start_index);

        while (!open_list.empty()) {
            unsigned int current_index = open_list.top().second;
            unsigned int current_x = current_index % width_;
            unsigned int current_y = current_index / width_;
            open_list.pop();

            if (current_index == goal_index) {
                return reconstructPath(came_from, current_index);
            }

            for (const auto& neighbor_index : getNeighbors(current_x, current_y)) {
                unsigned int neighbor_x = neighbor_index % width_;
                unsigned int neighbor_y = neighbor_index / width_;
                double tentative_g_cost = g_cost[current_index] + distance(current_x, current_y, neighbor_x, neighbor_y);

                if (tentative_g_cost < g_cost[neighbor_index]) {
                    g_cost[neighbor_index] = tentative_g_cost;
                    double f_cost = tentative_g_cost + heuristic(neighbor_x, neighbor_y, goal_x, goal_y);
                    open_list.emplace(f_cost, neighbor_index);
                    came_from[neighbor_index] = current_index;
                }
            }
        }

        return std::vector<unsigned int>(); // Return empty path if no path found
    }

std::vector<unsigned int> AStarPlanner::getNeighbors(unsigned int x, unsigned int y) {
    std::vector<unsigned int> neighbors;

    int clearance_cells = std::ceil(0.2 / resolution_); 
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
 
               int nx = static_cast<int>(x) + dx;
               int ny = static_cast<int>(y) + dy;
             

            if (nx >= clearance_cells && ny >= clearance_cells && 
                nx < static_cast<int>(width_) - clearance_cells && 
                ny < static_cast<int>(height_) - clearance_cells) {

                double cost = costmap_->getCost(nx, ny);
                if (cost >= 0 && cost <= 100) {
                        neighbors.push_back(ny * width_ + nx);
                    }
            }
        }
    }
    return neighbors;
}

    std::vector<unsigned int> AStarPlanner::reconstructPath(const std::vector<unsigned int>& came_from, unsigned int current_index) {
    std::vector<unsigned int> path;
    while (current_index != came_from[current_index]) { 
        path.insert(path.begin(), current_index);       
        current_index = came_from[current_index];
    }
    path.insert(path.begin(), current_index); 
    return path;
    }


    double AStarPlanner::potentialFieldCost(unsigned int x, unsigned int y) const {
        double cost = 0.0;
        for (int dx = -3; dx <= 3; ++dx) {
            for (int dy = -3; dy <= 3; ++dy) {
                int nx = static_cast<int>(x) + dx;
                int ny = static_cast<int>(y) + dy;
                if (nx >= 0 && ny >= 0 && nx < static_cast<int>(width_) && ny < static_cast<int>(height_)) {
                    double distance_to_obstacle = std::hypot(dx, dy);
                    if (distance_to_obstacle > 0.0) {
                        double obstacle_cost = costmap_->getCost(nx, ny);
                        if (obstacle_cost == costmap_2d::LETHAL_OBSTACLE) {
                            cost += 3.0 / distance_to_obstacle;  
                        }
                    }
                }
            }
        }
        return cost;
    }


    double AStarPlanner::heuristic(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const {
        double euclidean_distance = std::hypot(static_cast<double>(x2 - x1), static_cast<double>(y2 - y1));
        double potential_cost = potentialFieldCost(x1, y1);
        return euclidean_distance + potential_cost;
    }

    double AStarPlanner::distance(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const {
        return std::hypot(static_cast<double>(x2 - x1), static_cast<double>(y2 - y1));
    }

    void AStarPlanner::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const {
        wx = origin_x_ + mx * resolution_;
        wy = origin_y_ + my * resolution_;
    }

    std::vector<unsigned int> AStarPlanner::cudaAStarSearch(unsigned int start_x, unsigned int start_y, 
                                                            unsigned int goal_x, unsigned int goal_y){

        unsigned int start_index = start_y * width_ + start_x;
        unsigned int goal_index = goal_y * width_ + goal_x;

        std::vector<double> g_cost(width_ * height_, std::numeric_limits<double>::infinity());
        std::vector<unsigned int> came_from(width_ * height_, 0);
        std::priority_queue<std::pair<double, unsigned int>, std::vector<std::pair<double, unsigned int>>, std::greater<std::pair<double, unsigned int>>> open_list;

        g_cost[start_index] = 0.0;
        came_from[start_index] = start_index;
        open_list.emplace(heuristic(start_x, start_y, goal_x, goal_y), start_index);

        double* d_g_cost;
        double* d_open_list_costs;
        unsigned int* d_open_list_indices;
        int* d_open_list_size;
        unsigned int* d_came_from;

        int total_cells = width_ * height_;
        int size = 0;
        int h_open_list_size = 0;
        int* list_size;
        double h = heuristic(start_x, start_y, goal_x, goal_y);

        std::vector<double> f_cost(8);
        std::vector<unsigned int> h_neighbor_indices(total_cells);
        
        unsigned char* d_costmap;
        std::vector<unsigned char> h_costmap(width_ * height_);
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                h_costmap[y * width_ + x] = costmap_->getCost(x, y);
            }
        } 

        cudaMalloc(&d_g_cost, sizeof(double) * total_cells);
        cudaMalloc(&d_open_list_costs, sizeof(double) * 8);
        cudaMalloc(&d_open_list_indices, sizeof(unsigned int) * total_cells);
        cudaMalloc(&d_open_list_size, sizeof(int));
        cudaMalloc(&d_came_from, sizeof(unsigned int) * total_cells);
        cudaMalloc(&d_costmap, sizeof(unsigned char) * total_cells);

        cudaMemset(d_g_cost, std::numeric_limits<double>::infinity(), sizeof(double));
        cudaMemset(d_open_list_costs, h, sizeof(double));
        cudaMemset(d_open_list_indices, start_index, sizeof(unsigned int));
        cudaMemset(d_open_list_size, 0, sizeof(int));
        cudaMemset(d_came_from, 0, sizeof(unsigned int));

        cudaMemcpy(d_g_cost, g_cost.data(), sizeof(double) * total_cells, cudaMemcpyHostToDevice);
        cudaMemcpy(d_open_list_costs, f_cost.data(), sizeof(double) * 8, cudaMemcpyHostToDevice);
        cudaMemcpy(d_open_list_indices, h_neighbor_indices.data(), sizeof(unsigned int) * total_cells, cudaMemcpyHostToDevice);
        cudaMemcpy(d_open_list_size, &size, sizeof(int), cudaMemcpyHostToDevice);
        cudaMemcpy(d_came_from, came_from.data(), sizeof(unsigned int) * total_cells, cudaMemcpyHostToDevice);
        cudaMemcpy(d_costmap, h_costmap.data(), sizeof(unsigned char) * width_ * height_, cudaMemcpyHostToDevice);


        while (!open_list.empty()) {
            unsigned int current_index = open_list.top().second;
            unsigned int current_x = current_index % width_;
            unsigned int current_y = current_index / width_;
            open_list.pop();

            if (current_index == goal_index) {
                ROS_WARN("Reached goal at index %d", goal_index);
                ROS_WARN("came_from[goal_index] = %u", came_from[goal_index]);
                return reconstructPath(came_from, current_index);
            }

            cudaAStar(
                current_x,
                current_y,
                current_index,
                8,                     
                d_costmap,
                resolution_,
                width_,
                height_,
                d_g_cost,
                d_came_from,
                goal_x,
                goal_y,
                d_open_list_costs,
                d_open_list_indices,
                d_open_list_size,
                &h_open_list_size);    
            
            cudaDeviceSynchronize(); 

            cudaMemcpy(came_from.data(), d_came_from, sizeof(unsigned int) * total_cells, cudaMemcpyDeviceToHost);
            cudaMemcpy(f_cost.data(), d_open_list_costs, sizeof(double), cudaMemcpyDeviceToHost);
            cudaMemcpy(h_neighbor_indices.data(), d_open_list_indices, sizeof(int) * total_cells, cudaMemcpyDeviceToHost);
            cudaMemcpy(&h_open_list_size, d_open_list_size, sizeof(int), cudaMemcpyDeviceToHost);

            list_size = &h_open_list_size;

            //ROS_WARN("in .cpp, size:%d", *list_size);

            for (int i = 0; i < came_from.size(); ++i) {
                if (came_from[i] != 0) {
                    ROS_WARN("came_from[%d] = %u", i, came_from[i]);
                 }
                }

            for (int i = 0; i < *list_size; ++i) {
                open_list.emplace(f_cost[i], h_neighbor_indices[i]);
            }

            
        }

        cudaFree(d_g_cost);
        cudaFree(d_open_list_costs);
        cudaFree(d_open_list_indices);
        cudaFree(d_open_list_size);
        cudaFree(d_came_from);
        cudaFree(d_costmap);

        return std::vector<unsigned int>();
    
    }
};
