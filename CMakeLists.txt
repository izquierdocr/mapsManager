cmake_minimum_required(VERSION 2.8)
project(mapsmanager)
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(mapsmanager main.cpp)
target_link_libraries( mapsmanager ${OpenCV_LIBS} )
