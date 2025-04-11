#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>
#include <cuda_runtime.h>

extern "C" void launchNearestNeighborKernel(
    const float* d_src, const float* d_tgt, int N, int M, int* d_indices);

Eigen::Matrix4f runICPCUDA(
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
        launchNearestNeighborKernel(d_src, d_tgt, N, M, d_indices);
        cudaMemcpy(indices.data(), d_indices, sizeof(int) * N, cudaMemcpyDeviceToHost);

        std::vector<Eigen::Vector3f> src_pts, tgt_pts;
        for (int i = 0; i < N; ++i) {
            src_pts.emplace_back(src_points[3 * i], src_points[3 * i + 1], src_points[3 * i + 2]);
            int j = indices[i];
            tgt_pts.emplace_back(tgt_points[3 * j], tgt_points[3 * j + 1], tgt_points[3 * j + 2]);
        }

        Eigen::Vector3f mu_src = Eigen::Vector3f::Zero();
        Eigen::Vector3f mu_tgt = Eigen::Vector3f::Zero();
        for (int i = 0; i < N; ++i) {
            mu_src += src_pts[i];
            mu_tgt += tgt_pts[i];
        }
        mu_src /= N;
        mu_tgt /= N;

        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        for (int i = 0; i < N; ++i) {
            H += (src_pts[i] - mu_src) * (tgt_pts[i] - mu_tgt).transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
        Eigen::Vector3f t = mu_tgt - R * mu_src;

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;

        total_transform = T * total_transform;

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
