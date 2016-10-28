cmake_minimum_required(VERSION 2.8.3)
project(gocator_range)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")

if(CMAKE_VERSION VERSION_LESS "2.8.11")
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

## Find catkin macros and libraries
find_package(PkgConfig)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
#cmake_modules
  cv_bridge
  geometry_msgs
  roscpp
  rostime
  sensor_msgs
  std_msgs
)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)

##set paths and variables for GoSDK and kAPI libraries#set paths and variables for GoSDK and kAPI libraries
set(Go_SDK      ${PROJECT_SOURCE_DIR}/../gocator_bridge/GoSDK/libGoSdk.so)
set(kApi        ${PROJECT_SOURCE_DIR}/../gocator_bridge/GoSDK/libkApi.so)

#set include directories for GoSDK and kApi directories
set(gosdk_INCLUDE_DIR    ${PROJECT_SOURCE_DIR}/../gocator_bridge/include/gocator_bridge/GoSdk)
set(kApi_INCLUDE_DIR     ${PROJECT_SOURCE_DIR}/../gocator_bridge/include/gocator_bridge/kApi)

find_library(GOSDK_LIBRARIES Go_SDK kApi "Go_SDK and kApi Libraries")

find_library(GOSDK_LIBRARIES Go_SDK kApi "Go_SDK and kApi Libraries")

#Check to be sure libraries are correctly set

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge geometry_msgs roscpp rostime sensor_msgs std_msgs gocator_bridge
  DEPENDS system_lib
)

###########
## Build ##
###########

# include_directories(include)
include_directories(
  include
#  ${gocator_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRS/}
  ${gosdk_INCLUDE_DIR}
  ${kApi_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(gocator_profiler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(gocator_range src/gocator_range.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(gocator_range ${${PROJECT_NAME}_EXPORTED_TARGETS} #${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
  target_link_libraries(gocator_range ${catkin_LIBRARIES} 
                                        ${Go_SDK} 
                                        ${kApi}
                                        ${OpenCV_LIBRARIES}
 )


