cmake_minimum_required(VERSION 2.4.6)
project(monocular_pose_estimator_lib)

find_package(catkin REQUIRED COMPONENTS cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS eigen opencv
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB SOURCEFILES "src/*.cpp")

add_library( ${PROJECT_NAME} SHARED ${SOURCEFILES})
add_dependencies( ${PROJECT_NAME} SHARED ${catkin_EXPORTED_TARGETS} ) 
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})
