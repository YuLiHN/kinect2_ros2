cmake_minimum_required(VERSION 3.8)
project(kinect2_bridge)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # Check for c++11 support
# INCLUDE(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# IF(COMPILER_SUPPORTS_CXX11)
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# ELSEIF(COMPILER_SUPPORTS_CXX0X)
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
# ELSE()
#   MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# ENDIF()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(freenect2 REQUIRED HINTS "$ENV{HOME}/freenect2")
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(compressed_depth_image_transport REQUIRED)
find_package(kinect2_registration REQUIRED)
find_package(OpenCV REQUIRED)
find_package(depth_image_proc REQUIRED)




###########
# build #
###########
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)


include_directories(include
  ${Boost_INCLUDE_DIRS} # catkin_INCLUDE_DIRS
  ${OpenCV_INCLUDE_DIRS}
  ${freenect2_INCLUDE_DIRS}
  ${kinect2_registration_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${CMAKE_CURRENT_SOURCE_DIR}/include/internal"
)

if(DEPTH_REG_OPENCL)
  add_definitions(-DDEPTH_REG_OPENCL)
endif()
if(DEPTH_REG_CPU)
  add_definitions(-DDEPTH_REG_CPU)
endif()
add_definitions(-DK2_CALIB_PATH="${PROJECT_SOURCE_DIR}/data/")

add_executable(kinect2_bridge src/kinect2_bridge.cpp)
ament_target_dependencies(kinect2_bridge
  compressed_depth_image_transport
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  rclcpp
  std_msgs
  OpenCV
  freenect2
  kinect2_registration
)
target_link_libraries(kinect2_bridge
  ${Boost_SYSTEM_LIBRARY}
  ${OpenCV_LIBRARIES}
  ${freenect2_LIBRARY}
  ${kinect2_registration_LIBRARY}
)
ament_export_include_directories(include)
ament_export_dependencies(kinect2_registration)


###########
# install #
###########
install(TARGETS kinect2_bridge
  DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION include

)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
# install(FILES
#   nodelet_plugins.xml
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


ament_package()