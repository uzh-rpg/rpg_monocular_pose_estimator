cmake_minimum_required(VERSION 2.8.3)
project(monocular_pose_estimator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(OpenCV REQUIRED)

cs_add_executable(monocular_pose_estimator
    src/node.cpp src/monocular_pose_estimator.cpp
)
target_link_libraries(monocular_pose_estimator ${OpenCV_LIBRARIES})

cs_add_library(monocular_pose_estimator_nodelet
    src/nodelet.cpp src/monocular_pose_estimator.cpp
)
target_link_libraries(monocular_pose_estimator_nodelet ${OpenCV_LIBRARIES})

cs_install()
cs_export()
