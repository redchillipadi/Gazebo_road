# Gazebo_road
## Compile two files
First to move the libActorCollisionsPlugin.so to the gazebo plugins directory, the default directory is /usr/lib/x86_64-linux-gnu/gazebo-9/plugins.
Then run. 
## Install dependency
rosdep install --from-paths /home/yourusername/catkin/src --ignore-src
## Command
* Launch world  
roslaunch mrobot_gazebo my_gazebo3.launch
* Keyboard control commands  
roslaunch mrobot_teleop mrobot_teleop.launch
## View Camera
rqt_image_view
## Command control
Simple command statements in Human_detectio  
linear.x and angular.z these two parameters control the steering as well as the forward and backward movement of the trolley
