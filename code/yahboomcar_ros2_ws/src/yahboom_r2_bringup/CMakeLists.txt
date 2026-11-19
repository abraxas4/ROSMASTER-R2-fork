cmake_minimum_required(VERSION 3.16)
project(yahboom_r2_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/cmd_vel_stamper.py scripts/lidar_avoid.py scripts/door_patrol.py DESTINATION lib/${PROJECT_NAME})

ament_package()
