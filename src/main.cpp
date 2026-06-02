#include "Filterpipeline.hpp"
#include "VoxelFilter.hpp"
#include "OutlierFilter.hpp"
#include "LoadPcd.hpp"
#include <iostream>
#include <pcl/common/transforms.h>
#include "ICPRegistration.hpp"
#include "CloudViewer.hpp"
#include "FPFHReg.hpp"
#include "ConfigLoader.hpp"
#include <filesystem>

int main(int argc, char *argv[])
{
    std::string yaml_path;
    for (auto i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--config") == 0)
        {
            if (i + 1 >= argc)
            {
                spdlog::error("--config后面必须有配置文件路径!");
                return -1;
            }
            yaml_path = argv[i + 1];
        }
    }

    // 加载配置文件
    ConfigLoader cfg(yaml_path);
    auto outlier = cfg.getOutlierParam(); // outlier参数：邻居点数，标准差倍数
    auto fpfhParams = cfg.getFPFHParam(); // fpfh参数：法向量搜索半径，特征搜索半径
    auto icp = cfg.getICPParam();         // icp参数：最大迭代次数，最大匹配距离
    auto s_color = cfg.getSrcColorParam();
    auto t_color = cfg.getTgtColorParam();
    auto f_color = cfg.getRegColorParam();
    auto folder_path = cfg.getFolderPath();

    // 加载原始点云
    std::vector<std::string> pcd_path;                       // 存储文件夹下的全部点云路径
    CloudPtr global_map = pcl::make_shared<PointCloud>();    // 全局地图
    CloudPtr target_cloud = pcl::make_shared<PointCloud>();  // 目标点云
    CloudPtr current_cloud = pcl::make_shared<PointCloud>(); // 当前点云
    CloudPtr target_filtered = pcl::make_shared<PointCloud>();
    CloudPtr current_filtered = pcl::make_shared<PointCloud>();

    for (const auto path : std::filesystem::directory_iterator(folder_path))
    {
        if (path.is_regular_file() && path.path().extension() == ".pcd")
        {
            pcd_path.push_back(path.path().string());
        }
    }
    std::sort(pcd_path.begin(), pcd_path.end());
    if (pcd_path.size() < 2)
    {
        spdlog::error("至少需要两个以上的点云！");
        return -1;
    }
    if (!IoPcd::loadpcd(target_cloud, pcd_path[0]))
    {
        spdlog::error("无法打开目标点云！");
        return -1;
    }
    *global_map = *target_cloud; // 全局地图初始化

    // 初始帧下采样
    FilterPipeline src_pipeline;
    src_pipeline.addFilter(std::make_unique<VoxelFilter>(cfg.getVoxelParam()));
    src_pipeline.addFilter(std::make_unique<OutlierFilter>(outlier.mean_k, outlier.sd));
    src_pipeline.run(target_cloud, target_filtered);
    Eigen::Matrix4f global_trans = Eigen::Matrix4f::Identity(); // 全局变换矩阵
    for (size_t i = 1; i < pcd_path.size(); ++i)
    {
        // 第i帧点云下采样
        if (!IoPcd::loadpcd(current_cloud, pcd_path[i]))
        {
            return -1;
        }

        FilterPipeline tgt_pipeline;
        tgt_pipeline.addFilter(std::make_unique<VoxelFilter>(cfg.getVoxelParam()));
        tgt_pipeline.addFilter(std::make_unique<OutlierFilter>(outlier.mean_k, outlier.sd));
        tgt_pipeline.run(current_cloud, current_filtered);

        // 执行粗配准
        spdlog::info("开始第{}帧的配准", i);
        std::unique_ptr<RegistrationBase> coarse_reg = std::make_unique<FPFHReg>(fpfhParams[0], fpfhParams[1]);             // 法向量搜索半径，fpfh搜索半径
        std::unique_ptr<RegistrationBase> fine_reg = std::make_unique<ICPRegistration>(icp.icp_max_iter, icp.icp_max_dist); // 最大迭代次数，最大距离阈值
        Eigen::Matrix4f coarse_trans = coarse_reg->CoarseReg(current_filtered, target_filtered);                            // 粗配准矩阵
        Eigen::Matrix4f final_trans = Eigen::Matrix4f::Identity();                                                          // 每次配准最终的变换矩阵

        if (fine_reg->FineReg(current_filtered, target_filtered, coarse_trans, final_trans))
        {
            global_trans *= final_trans;
            spdlog::info("第{}帧配准成功", i);
            // std::stringstream ss;
            // ss << final_trans;
            // spdlog::info("最终矩阵为：\n{}", ss.str());
            // 结果导出
            CloudPtr reg_cloud = pcl::make_shared<PointCloud>();
            pcl::transformPointCloud(*current_cloud, *reg_cloud, global_trans);
            *global_map += *reg_cloud;
        }
        else
        {
            spdlog::error("第{}帧配准失败！", i);
        }
        target_filtered = current_filtered;
        target_cloud = current_cloud;
    }
    CloudViewer viewer("icp viewer");
    // viewer.addCloud(cloud, "source", s_color[0], s_color[1], s_color[2], 2);    // 原始点云，蓝
    // viewer.addCloud(cloud2, "target", t_color[0], t_color[1], t_color[2], 2);   // 点云，绿
    viewer.addCloud(global_map, "aligned", f_color[0], f_color[1], f_color[2]); // 配准后点云，红
    viewer.show();
    IoPcd::savepcd(global_map, "../data/result/result.pcd");

    return 0;
}