cmake_minimum_required(VERSION 2.8.3)
project(platform_rx_thread_test)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  platform_rx_msg
)

catkin_package(
  CATKIN_DEPENDS
    serial
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(rx_thread_test
  src/platform_rx_thread_test.cpp 
)

add_executable(debug_sb
  src/debug.cpp 
)

add_dependencies(debug_sb
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(rx_thread_test
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(rx_thread_test
  ${catkin_LIBRARIES}
)

target_link_libraries(debug_sb
  ${catkin_LIBRARIES}
)