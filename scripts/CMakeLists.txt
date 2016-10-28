cmake_minimum_required(VERSION 2.8.3)
project(scripts)

find_package(catkin REQUIRED COMPONENTS  roscpp  sensor_msgs)
find_package(catkin REQUIRED COMPONENTS roslaunch) #check .launch files for any errors
roslaunch_add_file_check(launch)


catkin_package(
  INCLUDE_DIRS include
)

set(maxon_lib  /usr/lib/libEposCmd.so)
set(maxon_lib2 /usr/lib/libftd2xx.so)

set(gocator_bridge_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/../gocator_bridge/include)

include_directories(
  ${catkin_INCLUDE_DIRS}
    ${gocator_bridge_INCLUDE_DIR}
)
add_executable(Epos src/HelloEposCmd.cpp)


target_link_libraries(Epos ${catkin_LIBRARIES}
						   ${maxon_lib}
						   ${maxon_lib2}
					)

