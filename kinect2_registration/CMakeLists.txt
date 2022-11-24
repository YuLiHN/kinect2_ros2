cmake_minimum_required(VERSION 3.8)
project(kinect2_registration)



# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################
# dependencies #
################
# additional cmake modules

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cmake_modules QUIET)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(Eigen3)
find_package(OpenCL)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

############################################################
## Check for modules being build based on found libraries ##
############################################################

if(EIGEN3_FOUND)
  message(STATUS "CPU based depth registration enabled")
  include_directories(${EIGEN3_INCLUDE_DIR})
  set(DEPTH_REG_CPU ON)
  add_definitions(-DDEPTH_REG_CPU)
else()
  message(STATUS "CPU based depth registration disabled")
  set(DEPTH_REG_CPU OFF)
endif()

if(OpenCL_FOUND)
  message(STATUS "OpenCL based depth registration enabled")
  set(EXPORTED_DEPENDENCIES OpenCL)

  if(UNIX AND NOT APPLE)
    # include(CheckOpenCLICDLoader)
    if(OpenCL_C_WORKS AND NOT OpenCL_CXX_WORKS)
      set(KINECT2_OPENCL_ICD_LOADER_IS_OLD 1)
      message(WARNING "Your libOpenCL.so is incompatible with CL/cl.h. Install ocl-icd-opencl-dev to update libOpenCL.so?")
    endif()
  endif()
  include_directories(${OpenCL_INCLUDE_DIRS})

  set(DEPTH_REG_OPENCL ON)
  add_definitions(-DDEPTH_REG_OPENCL)
else()
  message(STATUS "OpenCL based depth registration disabled")
  set(DEPTH_REG_OPENCL OFF)
endif()

include_directories(include)



if(NOT DEPTH_REG_CPU AND NOT DEPTH_REG_OPENCL)
  message(FATAL_ERROR "No registration method available!")
endif()


###########
## Build ##
###########

# Display additional files in qtcreator
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "${CMAKE_CURRENT_SOURCE_DIR}/include/internal"
)


if(DEPTH_REG_CPU)
  set(MODULES ${MODULES} src/depth_registration_cpu.cpp)
endif()

if(DEPTH_REG_OPENCL)
  add_definitions(-DREG_OPENCL_FILE="${PROJECT_SOURCE_DIR}/src/depth_registration.cl")
  set(MODULES ${MODULES} src/depth_registration_opencl.cpp)
  set(MODULE_LIBS ${MODULE_LIBS} OpenCL)
endif()

add_library(kinect2_registration SHARED src/kinect2_registration.cpp ${MODULES})
message(${MODULE_LIBS})
ament_target_dependencies(kinect2_registration
  rclcpp
  std_msgs
  OpenCV
  ${MODULE_LIBS}
)

#############
## Install ##
#############


## Mark executables and/or libraries for installation


install(TARGETS kinect2_registration
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(kinect2_registration)
ament_export_dependencies(${EXPORTED_DEPENDENCIES})
ament_package(CONFIG_EXTRAS cmake/kinect2_registration.cmake.in)
