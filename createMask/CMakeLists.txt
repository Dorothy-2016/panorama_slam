cmake_minimum_required(VERSION 2.8.3)
project(createMask)

find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES createMask
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
${OpenCV_INC}
)

add_executable(${PROJECT_NAME}_node src/createMask_node.cpp)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}  ${OpenCV_LIBS}
 )

