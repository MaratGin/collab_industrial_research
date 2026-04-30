cmake_minimum_required(VERSION 3.8)
project(gz_world_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gazebo_ros_actor_plugin REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY worlds launch config models meshes
  DESTINATION share/${PROJECT_NAME}
)


ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
        scripts/scrape_top_photos.py
        scripts/solo_safety_classifier.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# install(
#   DIRECTORY launch urdf config meshes worlds rviz scripts
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_python_install_package(${PROJECT_NAME})

# install(PROGRAMS
#         scripts/test_control.py
#         DESTINATION lib/${PROJECT_NAME})

ament_package()
