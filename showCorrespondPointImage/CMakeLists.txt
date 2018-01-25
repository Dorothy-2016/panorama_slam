cmake_minimum_required(VERSION 2.8.3)
project(showCorrespondPointImage)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -msse3 -std=c++11 -pthread -O3 -march=native -Wno-deprecated-declarations")
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
${OpenCV_INC}  ${PROJECT_SOURCE_DIR}
)
add_executable(${PROJECT_NAME}_node src/showCorrespondPointImage_node.cpp)
 target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
 ${OpenCV_LIBS}
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
 )


add_executable(ba_test  test/ba_test.cc)
target_link_libraries(ba_test ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so)
