cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(PointCloudPreprocessor LANGUAGES CXX)

# 1. 寻找PCL库（去掉不合理的版本号）
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

# 2. 声明目标，加上头文件方便编辑器识别
add_executable(pipeline_node
    src/main.cpp
    src/VoxelFilter.cpp
    src/OutlierFilter.cpp
    src/RegistrationBase.cpp
    src/ICPRegistration.cpp
    src/FPFHReg.cpp
    src/CloudViewer.cpp
    src/ConfigLoader.cpp
)

# 3. 强制指定C++14标准
target_compile_features(pipeline_node PUBLIC cxx_std_17)

# 4. 绑定头文件目录（去掉BUILD_INTERFACE）
target_include_directories(pipeline_node PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

# 5. 绑定宏定义
target_compile_definitions(pipeline_node PUBLIC ${PCL_DEFINITIONS})

# 6. 链接PCL库
target_link_libraries(pipeline_node PUBLIC ${PCL_LIBRARIES} yaml-cpp spdlog::spdlog)