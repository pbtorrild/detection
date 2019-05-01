cmake_minimum_required(VERSION 2.8.3)
project(detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs

)

find_package(OpenCV REQUIRED)

#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)
#generate_messages(DEPENDENCIES std_msgs)

catkin_package()

## Build talker and listener
include_directories(
# include
	${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



add_executable(talker src/Detection.cpp)
target_link_libraries(talker 	${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(talker detection_generate_messages_cpp)




## Generate added messages and services


## Declare a catkin package


##add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)
