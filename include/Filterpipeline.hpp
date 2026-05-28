#pragma once
#include "FilterBase.hpp"
#include <vector>
#include <memory>
#include <iostream>
#include <chrono> //计时

class FilterPipeline
{
public:
    void addFilter(std::unique_ptr<FilterBase> filter)
    {
        // std::move转移智能指针的所有权,filter-->filters_
        filters_.push_back(std::move(filter));
    }
    void run(const CloudPtr &start_cloud, CloudPtr &final_cloud)
    {
        // 当前处理的点云
        CloudPtr current_cloud = start_cloud;
        // std::cout << "开始执行pipeline，初始点数:" << start_cloud->points.size() << std::endl;

        // 遍历滤波器，逐个执行
        for (const auto &filter : filters_)
        {
            // 存储滤波后的点云
            CloudPtr next_cloud = pcl::make_shared<PointCloud>();
            // 计时
            auto start_time = std::chrono::steady_clock::now();
            filter->apply(current_cloud, next_cloud);
            // 计时结束
            auto end_time = std::chrono::steady_clock::now();
            // 计算时间（毫秒）
            auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            // 打印日志
            // std::cout << filter->getName() << "耗时：" << diff << "ms,剩余点数：" << next_cloud->points.size() << std::endl;
            // 把当前滤波结果交给下一个滤波器继续处理
            current_cloud = next_cloud;
        }
        final_cloud = current_cloud;
        // std::cout << "滤波执行完毕！\n";
    }

private:
    std::vector<std::unique_ptr<FilterBase>> filters_;
};