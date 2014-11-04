cmake_minimum_required(VERSION 2.8)
project( Saliency )
include_directories(/home/tairuichen/Desktop/GMR_Saliency/include)
link_directories(/home/tairuichen/Desktop/GMR_Saliency/include)

find_package( OpenCV REQUIRED )
add_library(imageProcess SLIC.cpp GMRsaliency.cpp)
add_executable( Saliency main.cpp )
target_link_libraries( Saliency imageProcess ${OpenCV_LIBS} )
