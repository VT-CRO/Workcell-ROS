cmake_minimum_required(VERSION 3.5)
project(mc2425_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Specify the message files relative to the package root
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AddPart.msg"
  "msg/PnPRemoval.msg"
  "msg/ShelfUpdate.msg"
  "srv/FileTransfer.srv"
)

ament_package()
