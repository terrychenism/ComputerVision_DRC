cmake_minimum_required(VERSION 2.8)
project( sphereMap )
include_directories(/home/tairuichen/Desktop/sphere_map/include)
link_directories(/home/tairuichen/Desktop/sphere_map/include)

find_package( OpenCV REQUIRED )
add_library(imageProcess ZMTY.cpp)
add_executable( sphereMap cylinder_map.cpp )
target_link_libraries( sphereMap imageProcess ${OpenCV_LIBS} )
