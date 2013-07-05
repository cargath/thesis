/**
 * @author Carsten Könemann
 */

// This is a ROS project
#include <ros/ros.h>

// Local headers
#include <thesis/config.h>
#include <thesis/semantic_map.h>

// We are working with TF
#include <tf/transform_listener.h>

// We want to subscribe to messages of this types
#include <thesis/ObjectStamped.h>

// We want to call these services
#include <thesis/DatabaseGetByID.h>

// We are going to publish object poses for debug purposes
#include <geometry_msgs/Pose.h>

// This node provides these services
#include <thesis/MappingGetAll.h>
#include <thesis/MappingGetByID.h>
#include <thesis/MappingGetByPosition.h>

// Transform listener
tf::TransformListener* transform_listener;

// Publish transformed object + camera poses for debug purposes
ros::Publisher object_pose_publisher;
ros::Publisher camera_pose_publisher;

// Reusable service clients
ros::ServiceClient db_get_by_type_client;

// The actual semantic map, storing recognized objects with map coordinates
SemanticMap semantic_map;

void object_callback(const thesis::ObjectStamped::ConstPtr& input)
{
  thesis::ObjectStamped transformed;
  transform_listener->waitForTransform(CAMERA_FRAME, MAP_FRAME, input->camera_pose.header.stamp, TF_TIMEOUT);
  // Transform recognized camera pose to map frame
  transform_listener->transformPose(MAP_FRAME, input->camera_pose, transformed.camera_pose);
  transformed.camera_pose.header.stamp = ros::Time(0);
  camera_pose_publisher.publish(transformed.camera_pose);
  // If available, transform recognized object pose to map frame
  if(!isnan(input->object_pose.pose.position.z))
  {
    transform_listener->transformPose(MAP_FRAME, input->object_pose, transformed.object_pose);
    transformed.object_pose.header.stamp = ros::Time(0);
    object_pose_publisher.publish(transformed.object_pose);
  }
  // Add transformed object to map
  thesis::DatabaseGetByID db_get_by_type_service;
  db_get_by_type_service.request.id = input->object_id;
  if(db_get_by_type_client.call(db_get_by_type_service))
  {
    float min_distance = (db_get_by_type_service.response.sample.width
                       +  db_get_by_type_service.response.sample.height) / 2;
    semantic_map.add(transformed, min_distance);
  }
  else
  {
    semantic_map.add(transformed);
    ROS_WARN("Failed to call service 'thesis_database/set_by_type'.");
    return;
  }
}

bool get_all(thesis::MappingGetAll::Request& request,
             thesis::MappingGetAll::Response& result)
{
  semantic_map.getAll(result.objects);
  return true;
}

bool get_by_type(thesis::MappingGetByID::Request& request,
                 thesis::MappingGetByID::Response& result)
{
  semantic_map.getByID(request.id, result.objects);
  return true;
}

bool get_by_position(thesis::MappingGetByPosition::Request& request,
                     thesis::MappingGetByPosition::Response& result)
{
  cv::Point3f p;
  p.x = request.position.x;
  p.y = request.position.y;
  p.z = request.position.z;
  semantic_map.getByPosition(p, result.objects);
  return true;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "thesis_mapping");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  // Create transform listener
  transform_listener = new tf::TransformListener();
  // Initialize reusable service clients
  ros::service::waitForService("thesis_database/get_by_type", -1);
  db_get_by_type_client = nh.serviceClient<thesis::DatabaseGetByID>("thesis_database/get_by_type");
  // Subscribe to relevant topics
  ros::Subscriber object_subscriber = nh.subscribe("thesis_recognition/objects", 1, object_callback);
  // Publish transformed object + camera poses for debug purposes
  object_pose_publisher = nh_private.advertise<geometry_msgs::PoseStamped>("object_pose", 1);
  camera_pose_publisher = nh_private.advertise<geometry_msgs::PoseStamped>("camera_pose", 1);
  // Advertise services
  ros::ServiceServer srv_all         = nh_private.advertiseService("all", get_all);
  ros::ServiceServer srv_by_type     = nh_private.advertiseService("by_type", get_by_type);
  ros::ServiceServer srv_by_position = nh_private.advertiseService("by_position", get_by_position);
  // Spin
  ros::spin();
  // Free memory
  delete transform_listener;
  // Exit
  return 0;
}
