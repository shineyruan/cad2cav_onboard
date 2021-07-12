cmake_minimum_required(VERSION 3.10)
project(object_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-fdiagnostics-color=always)  # colored output in Ninja build
set(CMAKE_C_FLAGS_DEBUG "-Wall -O0 -ggdb -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++17 -Wall -Werror -O0 -ggdb -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -Wall -O2")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cv_bridge
  image_transport
  sensor_msgs
  cartographer_ros_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OpenCV 4 REQUIRED)

find_package(CUDA)
if (CUDA_FOUND)
  message("\tFound CUDA for neural network inferencing!")
  add_definitions(-DUSE_CUDA)
else()
  message(WARNING, "\tCUDA not found. Use CPU for DNN inferencing...")
endif()


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES video_loader object_detector
 CATKIN_DEPENDS roscpp roslib cv_bridge image_transport sensor_msgs cartographer_ros_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

# include Apriltag-3.1.4 library
set(BUILD_PYTHON_WRAPPER OFF)
string(REPLACE " -Werror" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
add_subdirectory(include/apriltag-3.1.4)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

## Declare a C++ library
add_library(video_loader
  src/video_loader.cpp
)
add_library(object_detector
  src/object_detector.cpp
)
add_library(fps
  src/fps.cpp
)
add_library(landmark_processor
  src/landmark_processor.cpp
)
add_library(apriltag_detector
  src/apriltag_detector.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node node/object_detection_node.cpp)
add_executable(video_recording_node node/video_recording_node.cpp)
add_executable(image_publish_node node/image_publish_node.cpp)
add_executable(camera_calibration_node node/camera_calibration_node.cpp)

add_executable(image_subscribe_node test/image_subscriber_test.cpp)
add_executable(object_detection_test test/object_detection_test.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(object_detector PUBLIC
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(video_loader
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  object_detector
)
target_link_libraries(landmark_processor
  object_detector
  video_loader
)
target_link_libraries(apriltag_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  apriltag
)
target_link_libraries(${PROJECT_NAME}_node
  landmark_processor
)
target_link_libraries(video_recording_node
  video_loader
  fps
)
target_link_libraries(image_publish_node
  video_loader
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(camera_calibration_node
  video_loader
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(image_subscribe_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(object_detection_test 
  video_loader
  fps
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS
  ${PROJECT_NAME}_node
  video_recording_node
  image_publish_node
  camera_calibration_node
  image_subscribe_node
  object_detection_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS 
  video_loader
  object_detector
  fps
  landmark_processor
  apriltag_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING 
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY 
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_object_detection.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
