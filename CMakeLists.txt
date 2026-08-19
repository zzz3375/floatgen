cmake_minimum_required(VERSION 3.5)
project(floatgen)

find_package(ament_cmake REQUIRED)

ament_environment_hooks("hooks/resource_paths.dsv.in")
install(DIRECTORY meshes urdf launch gz config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/wind_farm_planner.py scripts/gz_tf_broadcaster.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
