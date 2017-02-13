cmake_minimum_required(VERSION 2.8.3)
project(wpi_ur5)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    cmake_modules
    geometric_shapes
    roscpp
    std_msgs
    genmsg
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)
find_package(Eigen REQUIRED)

find_package(OpenCV REQUIRED)

find_package(cv_bridge REQUIRED)

###########
## Build ##
###########

include_directories(/usr/include)
include_directories(/usr/include/opencv)
include_directories(/usr/include/opencv2)
include_directories(/opt/ros/indigo/share)

link_directories(/usr/lib)
link_directories(/usr/lib/x86_64-linux-gnu)
link_directories(/opt/ros/indigo/share)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${cv_bridge_LIBRARY_DIRS})

add_executable(move_group_ur5 src/move_group_ur5)
target_link_libraries(move_group_ur5 ${catkin_LIBRARIES} ${Boost_LIBRARIES}) # ${OpenCV_LIBRARIES})
install(TARGETS move_group_ur5 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(image2points src/image2points)
target_link_libraries(image2points ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES})