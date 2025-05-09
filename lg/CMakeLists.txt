cmake_minimum_required(VERSION 3.0.2)
project(lg)

# Find required packages (no custom message generation)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs  # Standard message types
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Declare a catkin package without custom message generation
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs  # Only use the standard ROS dependencies
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add executable for rangefinder
add_executable(rangefinder src/rangefinder.cpp)
target_link_libraries(rangefinder ${catkin_LIBRARIES})

# Add executable for serial node
add_executable(serial src/serial.cpp)
target_link_libraries(serial ${catkin_LIBRARIES})

# Add executable for incline calculator
add_executable(vector_incMatch src/vector_incMatch.cpp)
target_link_libraries(vector_incMatch ${catkin_LIBRARIES})

# Add executable for Maestro Control
add_executable(Maestro src/Maestro.cpp)
target_link_libraries(Maestro ${catkin_LIBRARIES})