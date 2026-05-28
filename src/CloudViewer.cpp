#include "CloudViewer.hpp"
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <vtkObject.h>

CloudViewer::CloudViewer(const std::string &window_name)
{
    vtkObject::GlobalWarningDisplayOff();
    viewer_ = std::make_unique<Visualizer>(window_name);
    viewer_->setBackgroundColor(0.05, 0.05, 0.05); // 深灰色
    viewer_->addCoordinateSystem(1.0f);            // 添加坐标轴
    viewer_->initCameraParameters();
}
void CloudViewer::addCloud(const CloudPtr &cloud, const std::string id, int r, int g, int b, int point_size)
{
    if (cloud->empty())
        return;
    // 创建颜色处理器
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(cloud, r, g, b);
    // 如果id已存在，则更新；不存在则添加
    if (!viewer_->updatePointCloud(cloud, color_handler, id))
    {
        viewer_->addPointCloud(cloud, color_handler, id);
    }
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
}
void CloudViewer::show()
{
    viewer_->resetCamera();
    spdlog::info("按q退出窗口...");
    while (!viewer_->wasStopped())
    {
        viewer_->spinOnce(100); // 100ms会刷新一次
    }
}
bool CloudViewer::wasStopped() const
{
    return viewer_->wasStopped();
}