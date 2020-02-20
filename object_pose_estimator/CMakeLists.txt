cmake_minimum_required(VERSION 3.5)
project(object_pose_estimator)

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

add_library(object_pose_estimator SHARED src/object_pose_estimator.cpp)
target_include_directories(object_pose_estimator PRIVATE include)
ament_target_dependencies(object_pose_estimator
                          rclcpp_components
                          rclcpp_lifecycle
                          std_msgs
                          pose_estimation_interface
                          zivid_interfaces
                          )           
                          
add_library(pose_listener SHARED src/pose_listener.cpp)
target_include_directories(pose_listener PRIVATE include)
ament_target_dependencies(pose_listener
                          rclcpp_components
                          geometry_msgs
                          Eigen3
                          )                          

add_executable(object_pose_test src/object_pose_test.cpp)
target_include_directories(object_pose_test PRIVATE include)
target_link_libraries(object_pose_test
                      object_pose_estimator
                      pose_listener
                      )

                      
install(DIRECTORY include/ DESTINATION include)

install(TARGETS 
        object_pose_estimator
        pose_listener
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS
        object_pose_test
        DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries( object_pose_estimator pose_listener )
ament_package()