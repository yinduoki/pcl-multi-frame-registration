#include "VoxelFilter.hpp"
#include <pcl/filters/voxel_grid.h>
//------函数实现----------
// 构造函数，把传入的leafsize赋值给成员变量leafsize
VoxelFilter::VoxelFilter(float leaf_size) : leaf_size_(leaf_size) {};
// VoxelFilter::VoxelFilter(float leaf_size){
//     leaf_size_ = leaf_size;
// }与上面相同

void VoxelFilter::apply(const CloudPtr &input, CloudPtr &output)
{
    // 创建体素过滤对象
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    vg.filter(*output);
}

// 返回当前滤波的名字
std::string VoxelFilter::getName() const
{
    return "VoxelGridFilter";
}
