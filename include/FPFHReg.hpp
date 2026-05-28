#pragma once
#include "RegistrationBase.hpp"
#include <spdlog/spdlog.h>

class FPFHReg : public RegistrationBase
{
public:
    FPFHReg(float normal_radius, float fpfh_radius);
    Eigen::Matrix4f CoarseReg(const CloudPtr &source, const CloudPtr &target) override;
    std::string getName() const override;

private:
    float normal_radius_; // 计算法向量搜索半径
    float fpfh_radius_;   // FPFH特征搜索半径（必须大于法向量半径）
};
