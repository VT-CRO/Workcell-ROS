# Manufacturing Workcell ROS Backend

Developed using ROS2

## Network

![network](https://github.com/VT-CRO/Workcell-Software/blob/main/images/network.jpg)

### Validated on

- Ubuntu 24.04
- ROS2 Jazzy

### Getting Started

Clone the repo

    git clone https://github.com/VT-CRO/Workcell-ROS.git
    cd Workcell-ROS

Build mc2425 and mc2425_msgs

    colcon build --packages-select mc2425 mc2425_msgs

Source the install

    source install/setup.bash

### Commands

To run the mainController node

    ros2 run mc2425 mainController

To run the printer node

    ros2 run mc2425 printer

To run the pickAndPlace node

    ros2 run mc2425 pickAndPlace

To add a part to the shelf

    ros2 run mc2425 addPrint --ros-args -p printer:=<INTEGER> -p height:=<FLOAT> -p name:=<STRING> -p author:=<STRING>

To remove a part from the shelf

    ros2 run mc2425 removePart --ros-args -p shelf:=<INTEGER>

To validate a part is in a shelf slot

    ros2 run mc2425 shelfCheck --ros-args -p shelf:=<INTEGER>

To request a file, ensure that the main controller and printer nodes are runnning

    ros2 run mc2425 requestGcode

### TODO

- Connection to Klipper nodes (Should each printer be its own node or should it just call Klipper commands through API?)
- Function to determine whether removal or plate movement should occur
- Removal node implementation
