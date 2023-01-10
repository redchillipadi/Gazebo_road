# Gazebo_road
# Compile two files
First to move the so plugins to the gazebo plugins directory, the default directory is /usr/lib/x86_64-linux-gnu/gazebo-9/plugins.

# Command
# launch world
roslaunch mrobot_gazebo my_gazebo3.launch
# Keyboard control commands
roslaunch mrobot_teleop mrobot_teleop.launch
# View Camera
rqt_image_view
