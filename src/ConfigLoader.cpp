#include "ConfigLoader.hpp"

ConfigLoader::ConfigLoader(const std::string file_yaml)
{
    try
    {
        config_ = YAML::LoadFile(file_yaml);
    }
    catch (const YAML::Exception &e)
    {
        throw std::runtime_error("无法读取yaml配置文件:" + std::string(e.what()));
    }
}

float ConfigLoader::getVoxelParam()
{
    float leaf_size = config_["voxel_filter"]["leaf_size"].as<float>(); // 体素过滤的体素大小
    return leaf_size;
}

OutlierParams ConfigLoader::getOutlierParam()
{
    OutlierParams params;
    params.mean_k = config_["outlierfilter"]["mean_k"].as<int>(); // 离群点过滤的邻居点个数
    params.sd = config_["outlierfilter"]["sd"].as<float>();       // 离群点过滤标准差倍数
    return params;
}

std::vector<float> ConfigLoader::getFPFHParam()
{
    std::vector<float> fpfhParam;
    float normal_radius = config_["FPFHReg"]["normal_radius"].as<float>(); // 计算法向量搜索半径
    float fpfh_radius = config_["FPFHReg"]["fpfh_radius"].as<float>();     // 计算fpfh搜索半径
    fpfhParam.push_back(normal_radius);
    fpfhParam.push_back(fpfh_radius);
    return fpfhParam;
}

IcpParams ConfigLoader::getICPParam()
{
    IcpParams params;
    params.icp_max_iter = config_["ICPReg"]["icp_max_iter"].as<int>();   // icp最大迭代次数
    params.icp_max_dist = config_["ICPReg"]["icp_max_dist"].as<float>(); // icp匹配距离阈值
    params.threshold = config_["ICPReg"]["threshold"].as<float>();
    return params;
}
std::vector<int> ConfigLoader::getSrcColorParam()
{
    std::vector<int> srcColorParam;
    srcColorParam = config_["source_color"].as<std::vector<int>>();
    return srcColorParam;
}
std::vector<int> ConfigLoader::getTgtColorParam()
{
    std::vector<int> tgtColorParam;
    tgtColorParam = config_["target_color"].as<std::vector<int>>();
    return tgtColorParam;
}
std::vector<int> ConfigLoader::getRegColorParam()
{
    std::vector<int> regColorParam;
    regColorParam = config_["reg_color"].as<std::vector<int>>();
    return regColorParam;
}

// 返回文件夹路径
std::string ConfigLoader::getFolderPath()
{
    std::string folder_path = config_["fold_path"].as<std::string>();
    return folder_path;
}
