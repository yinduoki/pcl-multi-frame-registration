#include "ICPRegistration.hpp"
#include "CloudViewer.hpp"
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <iostream>
#include <chrono>

using Normal = pcl::Normal;
using PointN = pcl::PointCloud<Normal>;
using NormalPtr = pcl::PointCloud<Normal>::Ptr;

ICPRegistration::ICPRegistration(int icp_max_iter, float icp_max_dist)
    : icp_max_iter_(icp_max_iter), icp_max_dist_(icp_max_dist) {}
bool ICPRegistration::FineReg(const CloudPtr &source, const CloudPtr &target, Eigen::Matrix4f &coarse_trans, Eigen::Matrix4f &final_trans)
{
    // 计时
    const auto t_start = std::chrono::steady_clock::now();
    // std::cout << "开始ICP精配准" << std::endl;
    auto coarse_cloud = pcl::make_shared<PointCloud>(); // 粗配准点云
    pcl::transformPointCloud(*source, *coarse_cloud, coarse_trans);

    // 精配准icp
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(coarse_cloud);
    icp.setInputTarget(target);
    icp.setMaximumIterations(icp_max_iter_);
    icp.setMaxCorrespondenceDistance(icp_max_dist_);
    icp.setTransformationEpsilon(1e-8); // 迭代收敛停止条件

    CloudPtr src_fine = pcl::make_shared<PointCloud>();
    icp.align(*src_fine);

    if (!icp.hasConverged())
    {
        std::cerr << "icp未能收敛！" << std::endl;
        return false;
    }
    Eigen::Matrix4f fine_trans = icp.getFinalTransformation(); // 精配准矩阵
    spdlog::info("完成精配准，适应度：{}", icp.getFitnessScore());

    // 计算最终矩阵
    final_trans = fine_trans * coarse_trans;
    const auto t_end = std::chrono::steady_clock::now();
    // std::cout << "精配准完成，总耗时："
    //           << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() << "ms\n";
    return true;
}

std::string ICPRegistration::getName() const
{
    return "icp";
}