cmake_minimum_required(VERSION 2.8)
project( DisplayImage )

#set(OpenCV_DIR "/home/jimcha/opencv/build")
find_package( OpenCV REQUIRED )
message(WARNING ${OpenCV_DIR})
message(WARNING ${OpenCV_DIR})

add_executable( DisplayImage DisplayImage.cpp )
target_link_libraries( DisplayImage ${OpenCV_LIBS} )

add_executable(disparity disparity_filters.cpp)
target_link_libraries(disparity ${OpenCV_LIBS})


add_executable(wls wls.cpp)
target_link_libraries(wls ${OpenCV_LIBS})

add_executable(track track.cpp)
target_link_libraries(track ${OpenCV_LIBS})
add_executable(videos videos.cpp)
target_link_libraries(videos ${OpenCV_LIBS})

add_executable(train_HOG train_HOG.cpp)
target_link_libraries(train_HOG ${OpenCV_LIBS})
