#pragma once // 防止头文件被重复包含
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
using CloudPtr = pcl::PointCloud<PointT>::Ptr;

//-----滤波基类（抽象基类、接口类）
// filterbase:所有点云滤波器的父类
class FilterBase
{
public:
    // 1、虚析构函数，子对象销毁时能正确调用父类的析构函数，防止内存泄露
    //=default :使用默认析构函数
    virtual ~FilterBase() = default;

    // 纯虚函数：父类不实现代码；所有子类必须重写这个函数
    // 作用：统一滤波接口：输入点云，输出滤波后的点云
    virtual void apply(const CloudPtr &input, CloudPtr &output) = 0;

    // 纯虚函数，返回滤波器的名字,const:不会修改当前这个类的成员变量
    virtual std::string getName() const = 0;
};