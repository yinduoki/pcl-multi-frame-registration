#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

struct OutlierParams
{
    int mean_k;
    float sd;
};
struct IcpParams
{
    int icp_max_iter;
    float icp_max_dist;
    float threshold;
};

class ConfigLoader
{
public:
    explicit ConfigLoader(const std::string file_yaml);
    float getVoxelParam();
    OutlierParams getOutlierParam();
    std::vector<float> getFPFHParam();
    IcpParams getICPParam();
    std::vector<int> getSrcColorParam();
    std::vector<int> getTgtColorParam();
    std::vector<int> getRegColorParam();
    std::string getFolderPath();

private:
    YAML::Node config_;
};