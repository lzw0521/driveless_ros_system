cmake_minimum_required(VERSION 2.8.3)
project(coverage_planner)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  roslint
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

catkin_package()

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/src
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
    /usr/include/eigen3
  ./include/rapidjson
  ./include/
)


roslint_cpp()
roslint_python()


add_library(${PROJECT_NAME}
        src/Turning.cpp
        src/Tracking.cpp
        src/MathCommon.cpp
        src/Planner.cpp
        src/CoveragePlanner.cpp
        )
add_executable(planner src/Planner.cpp)
add_executable(coverageplanner src/CoveragePlanner.cpp)
add_dependencies(planner
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})
add_dependencies(planner ${PROJECT_NAME}_gencpp)

target_link_libraries(planner
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_link_libraries(planner ${catkin_LIBRARIES})
target_link_libraries(coverageplanner
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
