## Robot Package Template

This is a GitHub template. You can make your own copy by clicking the green "Use this template" button.

It is recommended that you keep the repo/package name the same, but if you do change it, ensure you do a "Find all" using your IDE (or the built-in GitHub IDE by hitting the `.` key) and rename all instances of `my_bot` to whatever your project's name is.

Note that each directory currently has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has direcctories present for CMake to find). These example files can be removed if required (and the directories can be removed if `CMakeLists.txt` is adjusted accordingly).

This is the code that we will be using to launch the robot in rviz2 and gazebo
Create a ros pacakge names dev_ws `mkdir dev_ws`

Enter dev_ws `cd dev_ws` then `mkdir src` then `cd src`

Inside src clone the repository,  if unable to clone within wsl (mine was bringing issues) you can clone to windows then move it into wsl with `wsl cp -r /mnt/c/Users/user/my_bot /home/sandie/dev_ws/src/` replace sandie with ur wsl username

Open Vs code and install the WSL extension

Click F1 or Ctrl + Shift + P and search `WSL : Connect to WSL`

Once it opens choose the dev_ws package amoung the options to open


To open the robot in rviz:
Source ros  by typing `source install/setup.bash` 

Then launch the setup file by typing ` ros2 launch my_bot rsp.launch.py`

Open rviz in a different terminal `rviz2`

To control the wheels  run `ros2 run joint_state_publisher_gui joint_state_publisher_gui` in a different terminal

Rviz should open


For gazebo steps follow the next tutorial