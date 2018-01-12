cmake_minimum_required(VERSION 2.8.3)
project(showCorrespondPointImage)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
)
find_package(OpenCV 3 REQUIRED)
catkin_package(
)
include_directories(

  ${catkin_INCLUDE_DIRS}    ${OpenCV_INC}
)
add_executable(${PROJECT_NAME}_node src/showCorrespondPointImage_node.cpp)
 target_link_libraries(${PROJECT_NAME}_node
   ${OpenCV_LIBS}  ${catkin_LIBRARIES}
 )

