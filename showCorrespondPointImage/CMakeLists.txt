cmake_minimum_required(VERSION 2.8.3)
project(showCorrespondPointImage)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INC}  ${PROJECT_SOURCE_DIR}
)
add_executable(${PROJECT_NAME}_node src/showCorrespondPointImage_node.cpp)
 target_link_libraries(${PROJECT_NAME}_node
   ${OpenCV_LIBS}  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
 )

