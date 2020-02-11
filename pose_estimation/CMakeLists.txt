cmake_minimum_required(VERSION 3.5)
project(pose_estimation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pose_estimation_interface REQUIRED)

find_package(Zivid 1.8.0 COMPONENTS Core Vis3D PointCloudVisualizer REQUIRED)
find_package(OpenCV REQUIRED)      
find_package(xtl REQUIRED)
find_package(xtensor REQUIRED)  
find_package(BLAS REQUIRED) 
find_package(LAPACK REQUIRED)

add_library(chessboard_pose_estimator SHARED src/chessboard_pose_estimator.cpp)
target_include_directories(chessboard_pose_estimator PRIVATE include)
ament_target_dependencies(chessboard_pose_estimator
                          xtensor
                          Zivid
                          OpenCV
                          BLAS
                          LAPACK
                          )

add_library(pose_estimation SHARED src/pose_estimation.cpp)
target_include_directories(pose_estimation PRIVATE include)
ament_target_dependencies(pose_estimation
                          rclcpp_lifecycle
                          std_msgs
                          sensor_msgs
                          geometry_msgs
                          pose_estimation_interface
                          )
rclcpp_components_register_node(pose_estimation
                                PLUGIN
                                "pose_estimation::PoseEstimation"
                                EXECUTABLE
                                "pose_estimation_exec"
                                )


add_executable(test_script src/test_script.cpp)
target_include_directories(test_script PRIVATE include)
target_link_libraries(test_script
                      chessboard_pose_estimator
                      Zivid::Core
                      Zivid::Vis3D
                      xtensor
                      ${OpenCV_LIBS}
                      ${BLAS_LIBRARIES}
                      ${LAPACK_LIBRARIES}
                      )


install(DIRECTORY include/ DESTINATION include)

install(TARGETS 
        chessboard_pose_estimator
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS
        test_script
        DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()