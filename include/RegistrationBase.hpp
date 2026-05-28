#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense> //矩阵
#include <string>

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
using CloudPtr = pcl::PointCloud<PointT>::Ptr;

class RegistrationBase
{
public:
    virtual ~RegistrationBase() = default;

    // 纯虚函数，接受source，target，输出变换矩阵
    virtual Eigen::Matrix4f CoarseReg(const CloudPtr &source, const CloudPtr &target);
    virtual bool FineReg(const CloudPtr &source, const CloudPtr &target, Eigen::Matrix4f &coarse_trans, Eigen::Matrix4f &final_trans);
    virtual std::string getName() const;
};