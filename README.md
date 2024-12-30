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

Copy the klipper2ros.py communication file to the klipper directory

    cp klipper2ros.py $HOME/klipper

Install the requirements

    pip install -r requirements.txt

### Commands

To run the mainController node

    ros2 run mc2425 mainController

To run the printer node

    export PRINTER_ID=1 # This is an integer that is unique to a specific printer node
    # If you want to permanently save the printer ID use the following command:
    # echo "export PRINTER_ID=1" >> ~/.bashrc
    ros2 run mc2425 printer

To run the pickAndPlace node

    ros2 run mc2425 pickAndPlace

~~To add a part to the shelf~~ DEPRECATED 

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
