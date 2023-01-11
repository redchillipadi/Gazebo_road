# Gazebo_road

## Launching the simulation environment
1.move the libActorCollisionsPlugin.so to the gazebo plugins directory, the default directory is /usr/lib/x86_64-linux-gnu/gazebo-9/plugins.  
2.Install dependency  
rosdep install --from-paths /home/your_direction/src --ignore-src  
3.Compile  
catkin_make  
4.Launch  
roslaunch mrobot_gazebo my_gazebo3.launch.     
* if there is can’t locate node [mrobot_teleop.py] in package [mrobot_teleop]  
run chmod +x mrobot_teleop.py   
* Keyboard control commands.
roslaunch mrobot_teleop mrobot_teleop.launch
## View Camera
rqt_image_view
## Darnet_ros object detect
1. Download the darknet_ros package  
https://github.com/leggedrobotics/darknet_ros    
2. Change the subscribe camera topic in darknet_ros/darknet_ros/config/ros.yaml
3. Compile the package
4. Command 
roslaunch darknet_ros darknet_ros.launch
##  Follow mode   
//linear.x and angular.z these two parameters control the steering as well as the forward and backward movement of the trolley  
//After running darknet_ros run   
rosrun human_detecon  humandetection_node

