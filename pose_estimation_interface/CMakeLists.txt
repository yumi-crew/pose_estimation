cmake_minimum_required(VERSION 3.5)

project(pose_estimation_interface)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(pose_estimation_interface
                           "srv/EstimatePose.srv"
                           "srv/InitSurfaceMatch.srv"
                           DEPENDENCIES
                           builtin_interfaces
                           geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()