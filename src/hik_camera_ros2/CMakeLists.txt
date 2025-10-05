cmake_minimum_required(VERSION 3.8)
project(hik_camera_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)

set(MVS_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/external/MVS)
message(STATUS "Using MVS SDK from local path: ${MVS_SDK_PATH}")

find_path(MVS_INCLUDE_DIRS MvCameraControl.h
          PATHS ${MVS_SDK_PATH}/include)

find_library(MVS_LIBRARY libMvCameraControl.so
             PATHS ${MVS_SDK_PATH}/lib/64)

if(NOT MVS_INCLUDE_DIRS OR NOT MVS_LIBRARY)
    message(FATAL_ERROR "Local MVS SDK not found in ${MVS_SDK_PATH}! \n"
                        "Please ensure the 'include' and 'lib' folders are copied correctly.")
else()
    message(STATUS "Found MVS include directory: ${MVS_INCLUDE_DIRS}")
    message(STATUS "Found MVS library: ${MVS_LIBRARY}")
endif()

include_directories(${MVS_INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(hik_camera_node src/hik_camera_node.cpp)

target_include_directories(hik_camera_node PUBLIC include)
ament_target_dependencies(hik_camera_node
  rclcpp
  sensor_msgs
  image_transport
  camera_info_manager
  cv_bridge
)

target_link_libraries(hik_camera_node
  ${MVS_LIBRARY}
  pthread
)

install(TARGETS
  hik_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
