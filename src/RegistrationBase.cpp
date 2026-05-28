// RegistrationBase.cpp
#include "RegistrationBase.hpp"

Eigen::Matrix4f RegistrationBase::CoarseReg(const CloudPtr &, const CloudPtr &)
{
    throw std::runtime_error("CoarseReg() not implemented in this registrator");
    return Eigen::Matrix4f::Identity();
}

bool RegistrationBase::FineReg(const CloudPtr &, const CloudPtr &,
                               Eigen::Matrix4f &, Eigen::Matrix4f &)
{
    throw std::runtime_error("FineReg() not implemented in this registrator");
    return false;
}

std::string RegistrationBase::getName() const
{
    return "Base";
}