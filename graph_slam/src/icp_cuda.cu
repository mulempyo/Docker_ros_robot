#include "icp_cuda.h"
#include <cuda_runtime.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

extern "C" __global__
__global__ void findNearestNeighbors(
    const float* src, const float* tgt, int N, int M, int* indices) {
    
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;

    float min_dist = 1e10;
    int best_j = 0;

    float sx = src[3 * i];
    float sy = src[3 * i + 1];
    float sz = src[3 * i + 2];

    for (int j = 0; j < M; ++j) {
        float dx = sx - tgt[3 * j];
        float dy = sy - tgt[3 * j + 1];
        float dz = sz - tgt[3 * j + 2];
        float dist = dx * dx + dy * dy + dz * dz;
        if (dist < min_dist) {
            min_dist = dist;
            best_j = j;
        }
    }

    indices[i] = best_j;
}

extern "C" Eigen::Matrix4f runICPCUDA(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt_cloud,
    int max_iterations,
    float tolerance) {

    int N = src_cloud->size();
    int M = tgt_cloud->size();

    std::vector<float> src_points(N * 3);
    std::vector<float> tgt_points(M * 3);

    for (int i = 0; i < N; ++i) {
        src_points[3 * i]     = src_cloud->points[i].x;
        src_points[3 * i + 1] = src_cloud->points[i].y;
        src_points[3 * i + 2] = src_cloud->points[i].z;
    }
    for (int i = 0; i < M; ++i) {
        tgt_points[3 * i]     = tgt_cloud->points[i].x;
        tgt_points[3 * i + 1] = tgt_cloud->points[i].y;
        tgt_points[3 * i + 2] = tgt_cloud->points[i].z;
    }

    float *d_src, *d_tgt;
    int *d_indices;
    cudaMalloc(&d_src, sizeof(float) * N * 3);
    cudaMalloc(&d_tgt, sizeof(float) * M * 3);
    cudaMalloc(&d_indices, sizeof(int) * N);

    cudaMemcpy(d_src, src_points.data(), sizeof(float) * N * 3, cudaMemcpyHostToDevice);
    cudaMemcpy(d_tgt, tgt_points.data(), sizeof(float) * M * 3, cudaMemcpyHostToDevice);

    Eigen::Matrix4f total_transform = Eigen::Matrix4f::Identity();

    std::vector<int> indices(N);

    for (int iter = 0; iter < max_iterations; ++iter) {
        findNearestNeighbors<<<(N + 255) / 256, 256>>>(d_src, d_tgt, N, M, d_indices);
        cudaMemcpy(indices.data(), d_indices, sizeof(int) * N, cudaMemcpyDeviceToHost);

        std::vector<Eigen::Vector3f> src_pts, tgt_pts;
        for (int i = 0; i < N; ++i) {
            src_pts.push_back(Eigen::Vector3f(
                src_points[3 * i], src_points[3 * i + 1], src_points[3 * i + 2]
            ));
            int j = indices[i];
            tgt_pts.push_back(Eigen::Vector3f(
                tgt_points[3 * j], tgt_points[3 * j + 1], tgt_points[3 * j + 2]
            ));
        }

        // Compute centroids
        Eigen::Vector3f mu_src = Eigen::Vector3f::Zero();
        Eigen::Vector3f mu_tgt = Eigen::Vector3f::Zero();
        for (int i = 0; i < N; ++i) {
            mu_src += src_pts[i];
            mu_tgt += tgt_pts[i];
        }
        mu_src /= N;
        mu_tgt /= N;

        // Compute H matrix
        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        for (int i = 0; i < N; ++i) {
            H += (src_pts[i] - mu_src) * (tgt_pts[i] - mu_tgt).transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
        Eigen::Vector3f t = mu_tgt - R * mu_src;

        // Construct 4x4 transformation
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;

        total_transform = T * total_transform;

        // Apply transform to src_points
        for (int i = 0; i < N; ++i) {
            Eigen::Vector4f pt(src_points[3*i], src_points[3*i+1], src_points[3*i+2], 1.0f);
            pt = T * pt;
            src_points[3*i] = pt[0];
            src_points[3*i+1] = pt[1];
            src_points[3*i+2] = pt[2];
        }

        cudaMemcpy(d_src, src_points.data(), sizeof(float) * N * 3, cudaMemcpyHostToDevice);

        if (T.block<3,1>(0,3).norm() < tolerance) break;
    }

    cudaFree(d_src);
    cudaFree(d_tgt);
    cudaFree(d_indices);

    return total_transform;
}

