cmake_minimum_required(VERSION 3.0.2)
project(nuc)

find_package(catkin REQUIRED 
  COMPONENTS
    roscpp
    rospy
    shape_msgs
    nav_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  MeshWithFrame.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetNuc.srv
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  shape_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rospy
    shape_msgs
    nav_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/nuc.cpp
  src/nuc_node.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-O3")

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node 
  src/nuc.cpp
  src/nuc_node.cpp)

add_dependencies(${PROJECT_NAME}_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

install(
  TARGETS
    ${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

catkin_package()

