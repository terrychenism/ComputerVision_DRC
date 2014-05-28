#Set minimum version requered
cmake_minimum_required(VERSION 2.8.3)
#just to avoid the warning
if(COMMAND cmake_policy)
     cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

#set project name
project(perception_tracking)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  perception_common
)
include_directories(include ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uvc_camera_image
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

#Append path to the module path
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

#set the include directories
include_directories (${PROJECT_SOURCE_DIR}/./include	${OpenCV_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/./lib)


#libraries
add_library(tld_utils src/tld_utils.cpp)
add_library(LKTracker src/LKTracker.cpp)
add_library(ferNN src/FerNNClassifier.cpp)
add_library(tld src/TLD.cpp)

#executables
add_executable(tracking src/run_tld.cpp)
#target_link_libraries(tracking ${catkin_LIBRARIES})
#link the libraries
target_link_libraries(tracking tld LKTracker ferNN tld_utils ${OpenCV_LIBS} ${catkin_LIBRARIES})
#set optimization level 
set(CMAKE_BUILD_TYPE Release)

