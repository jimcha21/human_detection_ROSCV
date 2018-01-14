cmake_minimum_required(VERSION 2.8.3)
PROJECT (sgbm_ros)

FIND_PACKAGE(CUDA REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED core highgui imgproc contrib)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	cv_bridge
	image_transport
	message_generation
	sensor_msgs
)

SET(CUDA_NVCC_FLAGS "-arch=sm_32 -Wno-deprecated-gpu-targets" CACHE STRING "nvcc flags" FORCE)
SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
SET(LIB_TYPE STATIC) 

SET(STEREOSRCS 
	src/census.cu 
	src/matching_cost.cu 
	src/scan_scost.cu 
	src/winner_takes_all.cu 
	src/median_filter.cu 
	src/check_consistency.cu 
	src/cuda_utils.cu
	)


catkin_package( )
include_directories(
	${Boost_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

link_libraries(
  ${catkin_LIBRARIES}
)

CUDA_ADD_LIBRARY(sgm src/stereo_sgm.cpp ${STEREOSRCS} STATIC)

#set(SOURCES sample/image/stereosgm_image.cpp include/libsgm.h)
CUDA_ADD_EXECUTABLE(stereo_test sample/image/stereosgm_image.cpp ${CUDA_SRC})
TARGET_LINK_LIBRARIES(stereo_test sgm ${catkin_LIBRARIES} ${CUDA_LIBRARIES} ${CUDA_nppi_LIBRARY} ${OpenCV_LIBS})

CUDA_ADD_EXECUTABLE(image_sub sample/image/image_sub.cpp ${CUDA_SRC})
TARGET_LINK_LIBRARIES(image_sub sgm ${catkin_LIBRARIES} ${CUDA_LIBRARIES} ${CUDA_nppi_LIBRARY} ${OpenCV_LIBS})

CUDA_ADD_EXECUTABLE(image_subb sample/image/image_subb.cpp ${CUDA_SRC})
TARGET_LINK_LIBRARIES(image_subb sgm ${catkin_LIBRARIES} ${CUDA_LIBRARIES} ${CUDA_nppi_LIBRARY} ${OpenCV_LIBS})

