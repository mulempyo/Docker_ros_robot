#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

Eigen::Matrix4f runICPCUDA(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& src,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt,
    int max_iterations = 20,
    float tolerance = 1e-4);

