cmake_minimum_required(VERSION 2.8.3)
project(innovusion_zed)

# # 添加调试模式设置
set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-g)

# 添加发布模式
# set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
  pcl_ros
  pcl_conversions
  roslib
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
# find_package(OpenMP)
include_directories(${PCL_INCLUDE_DIRS})


# if(OpenMP_CXX_FOUND)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# endif()

# 输出PCL库的版本
message(STATUS "PCL_VERSION: ${PCL_VERSION}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_node
    CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport message_filters pcl_ros pcl_conversions roslib
)

add_executable(${PROJECT_NAME}_node 
  src/main.cpp
  src/parameter.cpp
  src/ros_node.cpp
  src/completion_model.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# # 添加OpenMP链接
# if(OpenMP_CXX_FOUND)
#     target_link_libraries(${PROJECT_NAME}_node OpenMP::OpenMP_CXX)
# endif()
