cmake_minimum_required(VERSION 2.8.3)
project(panorama_orb)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -msse3 -std=c++11 -pthread -O3 -march=native -Wno-deprecated-declarations")
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(OpenCV 3 REQUIRED)
catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INC}
  ${PROJECT_SOURCE_DIR}
)

add_executable(${PROJECT_NAME}_node
src/panorama_orb_node.cpp
src/ORBextractor.cpp
 )

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
)
