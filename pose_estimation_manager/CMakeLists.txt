cmake_minimum_required(VERSION 3.5)
project(pose_estimation_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(pose_estimation_interface REQUIRED)
find_package(zivid_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(pose_estimation_manager SHARED src/pose_estimation_manager.cpp)
target_include_directories(pose_estimation_manager PRIVATE include)
ament_target_dependencies(pose_estimation_manager
                          rclcpp_components
                          rclcpp_lifecycle
                          std_msgs
                          pose_estimation_interface
                          zivid_interfaces
                          )           
                          
add_library(pose_transformer SHARED src/pose_transformer.cpp)
target_include_directories(pose_transformer PRIVATE include)
ament_target_dependencies(pose_transformer
                          rclcpp_components
                          geometry_msgs
                          Eigen3
                          )                          
                      
install(DIRECTORY include/ DESTINATION include)

install(TARGETS 
        pose_estimation_manager
        pose_transformer
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)


ament_export_include_directories(include)
ament_export_libraries( pose_estimation_manager pose_transformer )
ament_package()