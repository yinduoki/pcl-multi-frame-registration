#pragma once
#include "FilterBase.hpp"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <spdlog/spdlog.h>

class IoPcd
{
public:
    // 静态方法，不需要实例化对象即可调用
    static bool loadpcd(const CloudPtr &cloud, const std::string &file_name)
    {
        if (pcl::io::loadPCDFile<PointT>(file_name, *cloud) == -1)
        {
            spdlog::error("无法加载文件：{}", file_name);
            return false;
        }
        spdlog::info("成功加载文件：{}", file_name);
        return true;
    }
    static bool savepcd(const CloudPtr &cloud, const std::string &file_name)
    {
        if (cloud->points.empty())
        {
            spdlog::info("{}为空，无法保存", file_name);
            return false;
        }
        pcl::io::savePCDFile<PointT>(file_name, *cloud);
        spdlog::info("成功保存文件：{}", file_name);
        return true;
    }
};
