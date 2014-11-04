cmake_minimum_required(VERSION 2.8)
project( Multi_TLD )


set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include_directories(/home/tairuichen/Desktop/Multi_TLD/include)
link_directories(/home/tairuichen/Desktop/Multi_TLD/include)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package( OpenCV REQUIRED )
add_executable( Multi_TLD multi_tld.cpp )
target_link_libraries( Multi_TLD ${OpenCV_LIBS} ${OpenMP_LIBS} )
