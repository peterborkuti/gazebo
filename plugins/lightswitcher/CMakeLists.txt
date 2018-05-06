cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(light_publisher_model_plugin SHARED LightPublisher.cc)
target_link_libraries(light_publisher_model_plugin ${GAZEBO_LIBRARIES})

add_library(light_subscriber_visual_plugin SHARED LightSubscriber.cc)
target_link_libraries(light_subscriber_visual_plugin ${GAZEBO_LIBRARIES})
