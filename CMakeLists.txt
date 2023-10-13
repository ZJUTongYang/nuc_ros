cmake_minimum_required(VERSION 3.12)
project(nuc_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  shape_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    std_srvs
)

include_directories(include)
include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS})

add_library(nuc_ros
  src/nuc.cpp
  src/nuc_main.cpp)
  
set_target_properties(nuc_ros PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

add_executable(nuc_node src/nuc_main.cpp)


set_target_properties(moveit_follow_group_default_capabilities PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
add_dependencies(moveit_follow_group_default_capabilities ${catkin_EXPORTED_TARGETS})

install(TARGETS follow_group list_follow_group_capabilities
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS moveit_follow_group_capabilities_base moveit_follow_group_default_capabilities
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(PROGRAMS
  scripts/load_map scripts/save_map DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
  default_capabilities_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # this test is flaky
  # add_rostest(test/test_cancel_before_plan_execution.test)
  add_rostest(test/test_check_state_validity_in_empty_scene.test)
endif()


