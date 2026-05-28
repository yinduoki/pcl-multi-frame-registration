#pragma once
#include "RegistrationBase.hpp"
#include <spdlog/spdlog.h>

class ICPRegistration : public RegistrationBase
{
public:
    ICPRegistration(int icp_max_iter, float icp_max_dist);
    bool FineReg(const CloudPtr &source, const CloudPtr &target, Eigen::Matrix4f &coarse_trans, Eigen::Matrix4f &final_trans) override;
    std::string getName() const override;

private:
    int icp_max_iter_;   // icp最大迭代次数
    float icp_max_dist_; // icp最大匹配距离；
};
