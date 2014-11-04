cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

project( Saliency_Filter )
include_directories(/home/tairuichen/Desktop/Saliency_Filter/include)
link_directories(/home/tairuichen/Desktop/Saliency_Filter/include)

find_package( OpenCV REQUIRED )
add_library(imageProcess filter.cpp saliency.cpp superpixel.cpp)
add_executable( saliency test_saliency.cpp )

target_link_libraries( saliency imageProcess ${OpenCV_LIBS} )

add_executable( superpixel test_superpixel.cpp )

target_link_libraries( superpixel imageProcess ${OpenCV_LIBS} )

