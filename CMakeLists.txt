cmake_minimum_required(VERSION 2.8.3)
project(human_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
FIND_PACKAGE(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  message_generation	)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mitsos
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(gpu_similarity src/gpu_similarity.cpp)
target_link_libraries(gpu_similarity ${catkin_LIBRARIES} )

add_executable(hog src/hog.cpp)
target_link_libraries(hog ${catkin_LIBRARIES} )

add_executable(svm_trainer src/svm_trainer.cpp)
target_link_libraries(svm_trainer ${catkin_LIBRARIES} )



