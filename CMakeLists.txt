cmake_minimum_required(VERSION 2.8.3)
project(bumper2pointcloud)
find_package(catkin REQUIRED COMPONENTS roscpp nodelet pluginlib sensor_msgs mower_msgs)

catkin_package(
   INCLUDE_DIRS src
   LIBRARIES bumper2pointcloud_nodelet
   CATKIN_DEPENDS roscpp nodelet pluginlib sensor_msgs mower_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(build_command_nodelet src/bumper2pointcloud.cpp)
add_dependencies(bumper2pointcloud_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(bumper2pointcloud_nodelet ${catkin_LIBRARIES})

install(TARGETS bumper2pointcloud_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
