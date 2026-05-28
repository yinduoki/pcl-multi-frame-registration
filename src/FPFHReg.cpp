#include "FPFHReg.hpp"
#include "CloudViewer.hpp"
#include "FilterBase.hpp"
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <iostream>
#include <chrono>

using Normal = pcl::Normal;
using PointN = pcl::PointCloud<Normal>;
using NormalPtr = pcl::PointCloud<Normal>::Ptr;

FPFHReg::FPFHReg(float normal_radius, float fpfh_radius)
    : normal_radius_(normal_radius), fpfh_radius_(fpfh_radius) {}

Eigen::Matrix4f FPFHReg::CoarseReg(const CloudPtr &source, const CloudPtr &target)
{
    const auto t_start = std::chrono::steady_clock::now();
    // std::cout << "开始FPFH粗配准" << std::endl;

    auto src_normals = pcl::make_shared<PointN>();
    auto tgt_normals = pcl::make_shared<PointN>();
    auto src_features = pcl::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
    auto tgt_features = pcl::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();

    // 计算法向量
    pcl::search::KdTree<PointT>::Ptr tree = pcl::make_shared<pcl::search::KdTree<PointT>>();
    pcl::NormalEstimation<PointT, Normal> ne;
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(normal_radius_);

    ne.setInputCloud(source);
    ne.compute(*src_normals);

    ne.setInputCloud(target);
    ne.compute(*tgt_normals);

    // 计算FPFH
    pcl::FPFHEstimation<PointT, Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(fpfh_radius_);

    fpfh.setInputCloud(source);
    fpfh.setInputNormals(src_normals);
    fpfh.compute(*src_features);

    fpfh.setInputCloud(target);
    fpfh.setInputNormals(tgt_normals);
    fpfh.compute(*tgt_features);

    const auto t_middle = std::chrono::steady_clock::now();
    // std::cout << "特征提取完成，用时：" << std::chrono::duration_cast<std::chrono::milliseconds>(t_middle - t_start).count() << "ms\n";

    // 粗配准
    pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sca_ia;
    auto front = pcl::make_shared<PointCloud>(); // 输出粗配准结果
    auto back = pcl::make_shared<PointCloud>();  // 输出反向配准结果
    // 正向配准
    sca_ia.setInputSource(source);
    sca_ia.setSourceFeatures(src_features);
    sca_ia.setInputTarget(target);
    sca_ia.setTargetFeatures(tgt_features);
    sca_ia.setMaximumIterations(80);          // 随机采样次数
    sca_ia.setMaxCorrespondenceDistance(1.5); // 距离阈值，变换后小于阈值才算内点
    sca_ia.align(*front);
    Eigen::Matrix4f front_trans = sca_ia.getFinalTransformation();
    auto front_score = sca_ia.getFitnessScore();

    Eigen::Matrix4f back_trans = front_trans;
    auto back_score = front_score; // 默认使用正向配准
    if (front_score > 0.2)
    {
        // 反向配准
        sca_ia.setInputSource(target);
        sca_ia.setSourceFeatures(tgt_features);
        sca_ia.setInputTarget(source);
        sca_ia.setTargetFeatures(src_features);
        sca_ia.setMaximumIterations(80);          // 随机采样次数
        sca_ia.setMaxCorrespondenceDistance(1.5); // 距离阈值，变换后小于阈值才算内点
        sca_ia.align(*back);
        back_trans = sca_ia.getFinalTransformation();
        back_score = sca_ia.getFitnessScore();
    }

    const auto t_end = std::chrono::steady_clock::now();

    spdlog::info("完成粗配准，适应度：{}", (front_score < back_score) ? front_score : back_score);
    // std::cout << "粗配准总耗时：" << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() << "ms\n";
    Eigen::Matrix4f final_trans = (front_score <= back_score) ? front_trans : back_trans.inverse();
    return final_trans;
}

std::string FPFHReg::getName() const
{
    return "sac_ia";
}