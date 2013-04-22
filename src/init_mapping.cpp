/**
 * @author Carsten Könemann
 */

// This is a ROS project
#include <ros/ros.h>

// We want to subscribe to messages of this types
#include <thesis/ObjectStamped.h>

// We are working with TF
#include <tf/transform_listener.h>

// Transform listener
tf::TransformListener transform_listener;

// Frame IDs
static const std::string CAMERA_FRAME = "/head_mount_kinect_rgb_optical_frame";
static const std::string MAP_FRAME    = "/map";

void object_callback(const thesis::ObjectStamped& input)
{
  //
  geometry_msgs::PoseStamped pose_camera,
                             pose_map;
  //
  ros::Time current_transform = ros::Time::now();
  transform_listener.waitForTransform(MAP_FRAME,
			                                CAMERA_FRAME,
			                                current_transform,
			                                ros::Duration(5.0));
  transform_listener.getLatestCommonTime(CAMERA_FRAME,
                                         MAP_FRAME,
				                                 current_transform,
				                                 NULL);  
  //
  pose_camera.header.stamp       = current_transform;
  pose_camera.header.frame_id    = CAMERA_FRAME;
  pose_camera.pose.position.x    = input.pose_stamped.pose.position.x;
  pose_camera.pose.position.y    = input.pose_stamped.pose.position.y;
  pose_camera.pose.position.z    = input.pose_stamped.pose.position.z;
  pose_camera.pose.orientation.x = input.pose_stamped.pose.orientation.x;
  pose_camera.pose.orientation.y = input.pose_stamped.pose.orientation.y;
  pose_camera.pose.orientation.z = input.pose_stamped.pose.orientation.z;
  pose_camera.pose.orientation.w = input.pose_stamped.pose.orientation.w;
  //
  transform_listener.transformPose(MAP_FRAME, pose_camera, pose_map);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "thesis_mapping");
  ros::NodeHandle nh;
  // Subscribe to relevant topics
  ros::Subscriber object_subscriber = nh.subscribe("thesis/recognition/objects", 1, object_callback);
  // Spin
  ros::spin();
  // Exit
  return 0;
}
