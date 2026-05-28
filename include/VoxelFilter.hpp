#pragma once
#include "FilterBase.hpp"

//----子类：体素滤波器-----
// public让父类的公开成员依旧公开
class VoxelFilter : public FilterBase
{
public:
    // 构造函数，传入体素的大小,explicit禁止隐式转换，单参数构造函数都得加
    explicit VoxelFilter(float leaf_size);

    // 重写父类纯虚函数，实现滤波功能,override告诉编译器重写父类函数，写错函数名会报错
    void apply(const CloudPtr &input, CloudPtr &output) override;

    std::string getName() const override;

private:
    float leaf_size_;
};