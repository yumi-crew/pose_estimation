cmake_minimum_required(VERSION 3.5)
project(pose_estimation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(
    CMAKE_MODULE_PATH 
    ${CMAKE_MODULE_PATH} 
    ${CMAKE_CURRENT_SOURCE_DIR}
)
if(DEFINED ENV{HALCONROOT} AND NOT $ENV{HALCONROOT} STREQUAL "")
  message("Using HALCONROOT " $ENV{HALCONROOT})
  STRING(REGEX REPLACE "\\\\" "/" HALCONROOT $ENV{HALCONROOT}) 
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
find_package(PCL 1.10 REQUIRED)
find_package(Halcon 18.11 REQUIRED)
find_package(OpenMP REQUIRED)

message(${HALCON_INC_DIRS} " " ${HALCON_LIBS})


add_library(chessboard_pose_estimator SHARED src/chessboard_pose_estimator.cpp)
target_include_directories(chessboard_pose_estimator PRIVATE include)
ament_target_dependencies(chessboard_pose_estimator
                          xtensor
                          Zivid
                          OpenCV
                          BLAS
                          LAPACK
                          )

add_library(opencv_surface_match SHARED src/opencv_surface_match.cpp)
target_include_directories(opencv_surface_match PRIVATE include)
target_link_libraries(opencv_surface_match libstdc++fs.a) #<filesystem> library
ament_target_dependencies(opencv_surface_match
                          OpenCV
                          )

add_library(halcon_surface_match SHARED src/halcon_surface_match.cpp)
target_include_directories(halcon_surface_match PUBLIC 
                          include
                          ${HALCON_INC_DIRS}                         
                          )
target_link_libraries(halcon_surface_match 
                        libstdc++fs.a
                        ${HALCON_LIBS}
                        )

          


add_library(pose_estimation_library SHARED src/pose_estimation.cpp)
target_include_directories(pose_estimation_library PUBLIC 
                          include
                          )
target_link_libraries(pose_estimation_library
                      chessboard_pose_estimator
                      opencv_surface_match
                      halcon_surface_match
                      ${PCL_LIBRARIES}
                      OpenMP::OpenMP_CXX
                      )
ament_target_dependencies(pose_estimation_library
                          rclcpp_components
                          rclcpp_lifecycle
                          std_msgs
                          sensor_msgs
                          geometry_msgs
                          pose_estimation_interface
                          xtensor
                          OpenCV
                          PCL
                          )
rclcpp_components_register_node(pose_estimation_library
                          PLUGIN
                          "pose_estimation::PoseEstimation"
                          EXECUTABLE
                          "pose_estimation")


add_executable(test_script src/test_script.cpp)
target_include_directories(test_script PRIVATE include)
target_link_libraries(test_script
                      opencv_surface_match
                      halcon_surface_match
                      chessboard_pose_estimator
                      Zivid::Core
                      Zivid::Vis3D
                      xtensor
                      ${OpenCV_LIBS}
                      ${BLAS_LIBRARIES}
                      ${LAPACK_LIBRARIES}
                      ${HALCON_LIBS}
                      )


install(DIRECTORY include/ DESTINATION include)

install(TARGETS 
        chessboard_pose_estimator
        pose_estimation_library
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS
        test_script
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()