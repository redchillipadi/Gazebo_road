#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <stdio.h>
#include <string.h>

ros::Publisher twist_pub;
//darknet_ros_msgs::BoundingBoxes data
 geometry_msgs::Twist msg;
void imageCallback(darknet_ros_msgs::BoundingBoxes data)
{
//  char str1[] = data.bounding_boxes[0].Class.c_str;
  char str2[]="";
   if (strcmp(data.bounding_boxes[0].Class.c_str(),str2)!=0)
{
    
    int a=(data.bounding_boxes[0].xmax-data.bounding_boxes[0].xmin)/2+data.bounding_boxes[0].xmin;
    int b=(data.bounding_boxes[0].ymax-data.bounding_boxes[0].ymin)/2+data.bounding_boxes[0].ymin;
    if (a<640)
   {
   // control action
        msg.angular.z = -0.1;
    // Publish message
   }
    if (a>640)
   {
   // control action
       msg.angular.z = 0.1;
    // Publish message
   }
   if(b<360)
   {
    msg.linear.x = 0.7;
   }
     if(b>360)
   {
    msg.linear.x = 0.7;
   }
    
    ROS_INFO("The vertical position of the centre point is %d",b);
    ROS_INFO("The horizontal position of the centre point is %d",a);

}
    ROS_INFO("vertical Speed is %f",msg.linear.x);
    ROS_INFO("horizontal Speed is %f",msg.linear.y);
    ROS_INFO("%s",data.bounding_boxes[0].Class.c_str());
}
int main(int argc, char** argv)
{
  // Initialise ROS
  ros::init(argc, argv, "move");
  ros::NodeHandle node;
  // Set up twist publisher
  
  ros::Rate loop_rate(10);
  twist_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Subscriber subscriber = node.subscribe("/darknet_ros/bounding_boxes", 1, imageCallback);
  
  
  while (ros::ok())
  {
    // Create message
    // Event loop
    msg.angular.z = 0.0;
    msg.linear.x = 0.0;
    ros::spinOnce();
    twist_pub.publish(msg);
    ROS_INFO("vertical Speed is %f",msg.linear.x);
    ROS_INFO("horizontal Speed is %f",msg.linear.y);
    loop_rate.sleep();
  }
  return 0;
}


