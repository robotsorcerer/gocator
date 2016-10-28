cmake_minimum_required(VERSION 2.8.3)
project(gocator_surface)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")

# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")

set(GLFW3_DIR ${PROJECT_SOURCE_DIR})

if(CMAKE_VERSION VERSION_LESS "2.8.11")
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

OPTION(ENABLE_OPENGL "Enable OpenGL support" ON)

if(ENABLE_OPENGL)
  find_package(OpenGL REQUIRED)
    if(OPENGL_FOUND)
    include_directories(
                        ${GLFW_INCLUDE_DIRS}
                        )
      find_package(X11 REQUIRED)
      ADD_DEFINITIONS(-DOPENGL_SUPPORT=1)
  ENDIF()
ENDIF(ENABLE_OPENGL)

## Find catkin macros and libraries
find_package(PkgConfig) 
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLEW REQUIRED)
find_package(gocator_bridge REQUIRED)
if(GLEW_FOUND)
  include_directories(${GLEW_INCLUDE_DIRS})
  link_libraries(${GLEW_LIBRARIES})
endif()

find_package(catkin REQUIRED COMPONENTS
                              cv_bridge
                              geometry_msgs
                              roscpp
                              rostime
                              sensor_msgs
                              std_msgs
                              roslaunch
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
find_package(PkgConfig) 
find_package(PCL REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gocator_bridge REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
if(GLEW_FOUND)
  include_directories(${GLEW_INCLUDE_DIRS})
  link_libraries(${GLEW_LIBRARIES})
endif()

find_package(catkin REQUIRED COMPONENTS cv_bridge geometry_msgs roscpp
                              rostime sensor_msgs std_msgs roslaunch)

roslaunch_add_file_check(launch)

#set include directories for GoSDK and kApi directories
set(gosdk_INCLUDE_DIR    ${PROJECT_SOURCE_DIR}/../gocator_bridge/include/gocator_bridge/GoSdk)
set(kApi_INCLUDE_DIR     ${PROJECT_SOURCE_DIR}/../gocator_bridge/include/gocator_bridge/kApi)
set(gocator_bridge_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/../gocator_bridge/include)

include_directories(
  include
  ${gocator_bridge_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${INCLUDE_DIRS/}
  ${gosdk_INCLUDE_DIR}
  ${kApi_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#set paths and variables for GoSDK and kAPI libraries
set(Go_SDK      ${PROJECT_SOURCE_DIR}/../gocator_bridge/GoSDK/libGoSdk.so)
set(kApi        ${PROJECT_SOURCE_DIR}/../gocator_bridge/GoSDK/libkApi.so)

set(maxon_lib2 /usr/lib/libftd2xx.so)
set(maxon_lib  /usr/lib/libEposCmd.so)

set(GoLIBS ${catkin_LIBRARIES} ${Go_SDK} ${kApi} ${GLFW_LIBRARIES} ${PCL_LIBRARIES} ${maxon_lib}   ${maxon_lib2})

catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge geometry_msgs roscpp rostime sensor_msgs std_msgs
  DEPENDS system_lib
)

message(STATUS  "gocator_bridge_include: " ${gocator_bridge_INCLUDE_DIR})

###########
## Build ##
###########



link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ executable
add_executable(gocator_surface src/gocator_surface.cpp)
add_executable(gocator_surfaceCB src/gocator_surfaceCB.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(gocator_surface ${GoLIBS} )
target_link_libraries(gocator_surfaceCB ${GoLIBS} )