cmake_minimum_required(VERSION 2.8.3)
project(monocular_pose_estimator_lib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
find_package(OpenCV REQUIRED)

file(GLOB SOURCEFILES "src/*.cpp")

cs_add_library( ${PROJECT_NAME} SHARED ${SOURCEFILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

cs_install()
cs_export()
