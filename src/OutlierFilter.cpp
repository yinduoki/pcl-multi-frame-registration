#include "OutlierFilter.hpp"
#include <pcl/filters/statistical_outlier_removal.h>

// 统计离群点移除滤波:删除点云中的噪点，孤立点。
OutlierFilter::OutlierFilter(int mean_k, float sd)
    : mean_k_(mean_k), sd_(sd) {}

void OutlierFilter::apply(const CloudPtr &input, CloudPtr &output)
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(input);
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(sd_);
    sor.filter(*output);
}

std::string OutlierFilter::getName() const
{
    return "StatisticalOutlierRemoval";
}