#include <cuda_runtime.h>
#include <vector>
#include <thrust/device_ptr.h>
#include <thrust/sort.h>
#include <ros/ros.h>

__device__ int getNeighborsDevice(
    unsigned int x,
    unsigned int y,
    unsigned int* neighbors,                    
    const unsigned char* costmap,  
    double resolution, 
    int width,
    int height)
{
    int count = 0;
    int clearance_cells = ceilf(0.2 / resolution);

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;

            int nx = static_cast<int>(x) + dx;
            int ny = static_cast<int>(y) + dy;

            if (nx >= clearance_cells && ny >= clearance_cells &&
                nx < (width - clearance_cells) &&
                ny < (height - clearance_cells)) {

                double cost = costmap[ny * width + nx];
                
                if (cost <= 250) {
                    neighbors[count++] = ny * width + nx;
                }
            }
        }
    }

    return count;  
}

__global__ void process_neighbors_kernel(
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
    int* open_list_size)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_points) return;

    unsigned int neighbors[8];
    int count = getNeighborsDevice(
        current_x, current_y,
        neighbors, 
        costmap,
        resolution,
        width,
        height);   

    for (int j = 0; j < count; ++j) {
        int neighbor_index = neighbors[j];
        int neighbor_x = neighbor_index % width;
        int neighbor_y = neighbor_index / width;

        double tentative_g = g_cost[current_index] + (static_cast<double>(neighbor_x - current_x) + static_cast<double>(neighbor_y - current_y));

        if (neighbor_index >= width * height) continue;

        if (tentative_g < g_cost[neighbor_index]) {
            g_cost[neighbor_index] = tentative_g;

            double f_cost = tentative_g + (static_cast<double>(neighbor_x - current_x) + static_cast<double>(neighbor_y - current_y));

            int insert_idx = atomicAdd(open_list_size, 1);

            open_list_costs[insert_idx] = f_cost;
            open_list_indices[insert_idx] = neighbor_index;

            came_from[neighbor_index] = current_index;
        }
    }
}

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
    double* d_open_list_costs,
    unsigned int* d_open_list_indices,
    int* open_list_size,
    int* h_open_list_size)
{
    int threads = 256;
    int blocks = (num_points + threads - 1) / threads;

    process_neighbors_kernel<<<blocks, threads>>>(
        current_x,
        current_y,
        current_index,
        num_points,
        costmap,
        resolution,
        width,
        height,
        g_cost,
        came_from,
        goal_x,
        goal_y,
        d_open_list_costs,
        d_open_list_indices,
        open_list_size);

    cudaDeviceSynchronize();     

    cudaMemcpy(h_open_list_size, open_list_size, sizeof(int), cudaMemcpyDeviceToHost);

    //ROS_WARN("open_list size = %d", *h_open_list_size);

    thrust::device_ptr<double> cost_ptr(d_open_list_costs);
    thrust::device_ptr<unsigned int> index_ptr(d_open_list_indices);
    thrust::sort_by_key(cost_ptr, cost_ptr + *h_open_list_size, index_ptr);
}
