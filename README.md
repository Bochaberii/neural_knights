## Robot Package Template

This is the code that we will be using to launch the robot in rviz2 and gazebo
Create a ros pacakge names dev_ws `mkdir dev_ws`

Enter dev_ws `cd dev_ws` then `mkdir src` then `cd src`

Inside src clone the repository

Open Vs code and install the WSL extension

Click F1 or Ctrl + Shift + P and search `WSL : Connect to WSL`

Once it opens choose the dev_ws package amoung the options to open

To open the robot in rviz:
Source ros by typing `source install/setup.bash`

Then launch the setup file by typing ` ros2 launch my_bot rsp.launch.py`

Open rviz in a different terminal `rviz2`

To control the wheels run `ros2 run joint_state_publisher_gui joint_state_publisher_gui` in a different terminal

Rviz should open

For gazebo steps follow the next tutorial
