cmake_minimum_required(VERSION 2.8.3)
project(guidance)

set(yaml-cpp_DIR "/home/ubuntu/Documents/yaml-cpp/build")
#message(FATAL_ERROR ${YAML_CPP_VERSION})
#message(WARNING ${yaml-cpp_DIR})
find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
	camera_info_manager
  tf
	yaml-cpp	
)

#set(OpenCV_DIR "/home/ubuntu/opencv/build")
find_package(OpenCV 2 REQUIRED core highgui)
#message( FATAL_ERROR ${OpenCV_VERSION})
add_message_files(
  FILES
  sonar_info.msg
)
generate_messages()

catkin_package(
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
	include
)

message("System is: " ${CMAKE_SYSTEM_PROCESSOR})
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    message("-- 32bit detected")
    link_directories(lib/x86)
  elseif (CMAKE_SIZEOF_VOID_P EQUAL 8)
    message("-- 64bit detected")
    link_directories(lib/x64)
  endif ()
else()
  message("-- Non-linux platform detected but sorry we do not support :D")
endif ()

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l" )
  message("-- " ${CMAKE_SYSTEM_PROCESSOR} " detected")
  link_directories(lib/XU3)
else ()
endif ()

link_libraries(
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  DJI_guidance
  usb-1.0
  yaml-cpp)

add_executable(guidanceNode
  src/GuidanceNode.cpp
  src/DJI_utility.cpp)

add_executable(laserscan_transform_toNao
  nodes/laserscan_transform_toNao.cpp
  src/DJI_utility.cpp)

add_executable(armarker_subscriber
  nodes/armarker_subscriber.cpp
	src/DJI_utility.cpp
	)
	
add_executable(maptomarker_tf
  nodes/maptomarker_tf.cpp
  src/DJI_utility.cpp)

add_executable(initialpose_pub 
  nodes/initialpose_pub.cpp
    src/DJI_utility.cpp
      )

add_executable(tf_publisher 
  nodes/tf_publisher.cpp
    src/DJI_utility.cpp
      )
	
add_executable(camerainfo_publisher
  nodes/camerainfo_publisher.cpp
	src/DJI_utility.cpp
)
target_link_libraries(camerainfo_publisher 
${catkin_LIBRARIES}
${OpenCV_LIBS}
${YAML_CPP_LIBRARIES}
#[[/usr/lib/x86_64-linux-gnu/libyaml-cpp.so]]
)
	
add_executable(allCamerasSonars_save
  src/allCamerasSonars_save.cpp
	src/DJI_utility.cpp
	)

add_executable(FullCameraSimpleNode 
  src/FullCameraSimpleNode.cpp
      src/DJI_utility.cpp
        )
					
add_executable(node 
	src/node.cpp
	src/DJI_utility.cpp
	)
			
add_executable(a 
	src/a.cpp
	src/DJI_utility.cpp
	)
	
	add_executable(bb 
	src/b.cpp
	src/DJI_utility.cpp
	)
			
#add_executable(camerasinfo_publisher 
#	src/camerasinfo_publisher.cpp
#	src/DJI_utility.cpp
#	)

	
add_executable(guidanceNodeTest
  src/GuidanceNodeTest.cpp)

add_executable(guidanceNodeCalibration
  src/GuidanceNodeCalibration.cpp
  src/DJI_utility.cpp)

add_executable(test_yaml
  src/test_yaml.cpp
  src/DJI_utility.cpp)
#  vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 :
