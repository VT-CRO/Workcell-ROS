# Manufacturing Workcell Backend
Developed using ROS2

### Validated on
- Ubuntu 24.04
- ROS2 Jazzy

### Getting Started
Clone the repo

    git clone https://github.com/VT-CRO/Workcell-Software.git
    cd Workcell-Software

Build mc2425 and mc2425_msgs

    colcon build --packages-select mc2425 mc2425_msgs

Source the install

    source install/setup.bash

To run the mainController node

    ros2 run mc2425 mainController

To add a part to the shelf

    ros2 run mc2425 addPrint --ros-args -p printer:=<INTEGER> -p height:=<FLOAT> -p name:=<STRING> -p author:=<STRING>
