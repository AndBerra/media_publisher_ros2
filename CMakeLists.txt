cmake_minimum_required(VERSION 3.8)
project(media_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate message interfaces from the .msg file
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Audio.msg"
)

# This is the standard macro that will correctly find and process
# the setup.py file for all Python installation tasks.
ament_package()