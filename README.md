# PCL 点云配准 Pipeline

基于 PCL（Point Cloud Library）的多帧点云配准与拼接系统，支持顺序读取多帧点云，通过粗配准 + 精配准的方式实现全局地图构建。

## 功能特性

- **多级滤波**：体素下采样 + 统计离群点过滤
- **粗配准**：基于 FPFH 特征的 SAC-IA 算法，采用双向配准策略
- **精配准**：ICP 迭代最近点算法
- **多帧拼接**：顺序配准并累积生成全局地图
- **可视化**：集成 PCL Viewer 实时查看配准结果

## 环境依赖

| 依赖 | 版本要求 |
|------|----------|
| CMake | >= 3.10 |
| PCL | >= 1.10 |
| yaml-cpp | - |
| spdlog | - |
| 编译器 | 支持 C++17 |

### Ubuntu 安装依赖

```bash
sudo apt install libpcl-dev libyaml-cpp-dev libspdlog-dev
```

## 快速开始

### 1. 编译

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 2. 准备数据

将待配准的 `.pcd` 点云文件放入 `data/` 目录，程序会按文件名顺序读取：

```
data/
├── 1.pcd    # 第一帧（基准帧）
├── 2.pcd    # 第二帧
├── 3.pcd    # ...
└── result/  # 输出目录
```

### 3. 运行

```bash
cd build
./pipeline_node --config ../config/default.yaml
```

配准结果保存至 `data/result/result.pcd`。

## 配置说明

配置文件位于 `config/default.yaml`，主要参数：

```yaml
voxel_filter:
  leaf_size: 0.06        # 体素大小（米），越大下采样越激进

outlierfilter:
  mean_k: 10             # 邻居点数量
  sd: 2.0                # 标准差倍数，越小过滤越激进

FPFHReg:
  normal_radius: 0.2     # 法向量搜索半径
  fpfh_radius: 0.5       # FPFH 特征搜索半径

ICPReg:
  icp_max_iter: 50       # 最大迭代次数
  icp_max_dist: 0.2      # 最大匹配距离

fold_path: /home/hdz/pcl_pipeline/data  # 输入数据路径
```

## 项目结构

```
pcl_pipeline/
├── CMakeLists.txt
├── config/
│   └── default.yaml          # 参数配置
├── data/
│   ├── *.pcd                 # 输入点云
│   └── result/               # 输出目录
├── include/
│   ├── FilterBase.hpp        # 滤波器基类
│   ├── VoxelFilter.hpp       # 体素滤波
│   ├── OutlierFilter.hpp     # 离群点过滤
│   ├── Filterpipeline.hpp    # 滤波管道
│   ├── RegistrationBase.hpp  # 配准基类
│   ├── FPFHReg.hpp           # FPFH 粗配准
│   ├── ICPRegistration.hpp   # ICP 精配准
│   ├── CloudViewer.hpp       # 可视化
│   ├── ConfigLoader.hpp      # 配置加载
│   └── LoadPcd.hpp           # 点云 IO
└── src/
    ├── main.cpp              # 主程序
    ├── VoxelFilter.cpp
    ├── OutlierFilter.cpp
    ├── FPFHReg.cpp
    ├── ICPRegistration.cpp
    ├── CloudViewer.cpp
    └── ConfigLoader.cpp
```

## 算法流程

```
输入点云序列
      |
      v
+-----------------+
|   体素滤波      |  下采样，减少点数
+--------+--------+
         |
         v
+-----------------+
|   离群点过滤    |  去除噪声点
+--------+--------+
         |
         v
+-----------------+
|  FPFH 粗配准    |  基于特征的初始对齐
+--------+--------+
         |
         v
+-----------------+
|   ICP 精配准    |  迭代优化变换矩阵
+--------+--------+
         |
         v
+-----------------+
|   累积拼接      |  变换并合并到全局地图
+--------+--------+
         |
         v
      全局地图
```

## License

MIT