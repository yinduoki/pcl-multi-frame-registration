#pragma once
#include "FilterBase.hpp"
#include <vector>
#include <memory>
#include <pcl/visualization/pcl_visualizer.h>
#include <spdlog/spdlog.h>

using Visualizer = pcl::visualization::PCLVisualizer;

class CloudViewer
{
public:
    explicit CloudViewer(const std::string &window_name);
    void addCloud(const CloudPtr &cloud, const std::string id, int r, int g, int b, int point_size = 1);
    // 启动窗口阻塞,直到窗口关闭
    void show();
    bool wasStopped() const;

private:
    std::unique_ptr<Visualizer> viewer_;
};