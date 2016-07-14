cmake_minimum_required(VERSION 2.8.3)
project(behavior_trees)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  roslib
  rospy
  std_msgs
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread program_options)

# Settings
add_definitions(-Wall -g -O0 -Wno-deprecated -static -Bstatic -std=gnu++0x)

#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

catkin_python_setup()

# Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  ROS.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bt_server
  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp roslib rospy std_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

file(GLOB_RECURSE SRC ${PROJECT_SOURCE_DIR}/src/client/*.cpp)

add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${catkin_LIBRARIES} ${Boost_LIBRARIES} X11)

add_executable(template_server src/server/template_server.cpp src/server/rosaction.cpp src/client/robot_config.cpp)
target_link_libraries(template_server ${catkin_LIBRARIES} ${Boost_LIBRARIES} X11)

add_library(bt_server STATIC ${PROJECT_SOURCE_DIR}/src/server/rosaction.cpp)

add_dependencies(behavior_trees  behavior_trees_generate_messages_cpp)
add_dependencies(template_server behavior_trees_generate_messages_cpp)
add_dependencies(bt_server       behavior_trees_generate_messages_cpp)
