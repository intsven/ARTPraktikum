cmake_minimum_required(VERSION 2.8.3)
project(aufgaben)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg nav_msgs tf volksbot)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(${catkin_INCLUDE_DIRS})
add_executable(talker src/odom_publisher.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
install(TARGETS talker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(listener src/odom_subscriber.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
install(TARGETS listener
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(amcl_listener src/amcl_subscriber.cpp)
target_link_libraries(amcl_listener ${catkin_LIBRARIES})
install(TARGETS amcl_listener
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(giovanni_simulator src/giovanni/main.cc)
target_link_libraries(giovanni_simulator ${catkin_LIBRARIES})
install(TARGETS giovanni_simulator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(robot_control src/giovanni/robot_control.cc)
target_link_libraries(robot_control ${catkin_LIBRARIES})
install(TARGETS robot_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(robot_control_amcl src/giovanni/robot_control_amcl.cc)
target_link_libraries(robot_control_amcl ${catkin_LIBRARIES})
install(TARGETS robot_control_amcl
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


