/**
 * @author Carsten Könemann
 */

// This is a ROS project
#include <ros/ros.h>

// Local headers
#include <thesis/config.h>
#include <thesis/semantic_map.h>

// We want to subscribe to messages of this types
#include <thesis/ObjectStamped.h>

// We are going to publish object poses for debug purposes
#include <geometry_msgs/Pose.h>

// We are working with TF
#include <tf/transform_listener.h>

// Transform listener
tf::TransformListener transform_listener;

// Publish transformed object poses for debug purposes
ros::Publisher object_pose_publisher;

// The actual semantic map, storing recognized objects with map coordinates
SemanticMap semantic_map;

void object_callback(const thesis::ObjectStamped::ConstPtr& input)
{
  // Transform recognized object to map frame
  thesis::ObjectStamped transformed;
  transform_listener.waitForTransform(CAMERA_FRAME, MAP_FRAME, input->object_pose.header.stamp, TF_TIMEOUT);
  transform_listener.transformPose(MAP_FRAME, input->object_pose, transformed.object_pose);
  transformed.object_pose.header.stamp = ros::Time(0);
  // Publish transformed object pose for debug purposes
  object_pose_publisher.publish(transformed.object_pose);
  // Add transformed object to map
  semantic_map.add(transformed);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "thesis_mapping");
  ros::NodeHandle nh("~");
  // Subscribe to relevant topics
  ros::Subscriber object_subscriber = nh.subscribe("thesis_recognition/objects", 1, object_callback);
  // Publish transformed object poses for debug purposes
  object_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("object_pose", 1);
  // Spin
  ros::spin();
  // Exit
  return 0;
}
