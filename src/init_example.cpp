/**
 * @author Carsten Könemann
 */

// This is a ROS project
#include <ros/ros.h>

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "thesis_example");
  ros::NodeHandle nh_private("~");
  
  // Spin
  ros::spin();
  // Exit
  return 0;
}
