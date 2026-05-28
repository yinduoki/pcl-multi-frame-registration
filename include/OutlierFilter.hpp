#pragma once
#include "FilterBase.hpp"

class OutlierFilter : public FilterBase
{
public:
    // 传入参数，邻居点数量和标准差倍数
    OutlierFilter(int mean_k, float sd);
    void apply(const CloudPtr &input, CloudPtr &output) override;
    std::string getName() const override;

private:
    int mean_k_;
    float sd_;
};