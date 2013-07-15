/**
 * @author Carsten Könemann
 */

// This is a ROS project
#include <ros/ros.h>

// Local headers
#include <thesis/semantic_map.h>

// We are working with TF
#include <tf/transform_listener.h>

// We want to subscribe to messages of this types
#include <thesis/ObjectStamped.h>

// We want to call these services
#include <thesis/DatabaseGetByID.h>

// This node provides these services
#include <thesis/MappingGetAll.h>
#include <thesis/MappingGetByID.h>
#include <thesis/MappingGetByPosition.h>

// Config parameters
std::string camera_frame,
            map_frame;
            
double      tf_timeout;

// Transform listener
tf::TransformListener* transform_listener;

// Reusable service clients
ros::ServiceClient db_get_by_type_client;

// The actual semantic map, storing recognized objects with map coordinates
SemanticMap semantic_map;

void object_callback(const thesis::ObjectStamped::ConstPtr& input)
{
  ROS_INFO("Mapping: Object callback.");
  thesis::ObjectStamped transformed;
  transform_listener->waitForTransform(camera_frame, map_frame, input->camera_pose.header.stamp, ros::Duration(tf_timeout));
  // Transform recognized camera pose to map frame
  transform_listener->transformPose(map_frame, input->camera_pose, transformed.camera_pose);
  transformed.camera_pose.header.stamp = ros::Time(0);
  // If available, transform recognized object pose to map frame
  if(!isnan(input->object_pose.pose.position.z))
  {
    ROS_INFO("Mapping: Object caught.");
    
    transform_listener->transformPose(map_frame, input->object_pose, transformed.object_pose);
    transformed.object_pose.header.stamp = ros::Time(0);
    
    std::cout << "Object position:         " << input->object_pose.pose.position.x         << ", " << input->object_pose.pose.position.y         << ", " << input->object_pose.pose.position.z         << std::endl;
    std::cout << "Transformed position:    " << transformed.object_pose.pose.position.x    << ", " << transformed.object_pose.pose.position.y    << ", " << transformed.object_pose.pose.position.z    << std::endl;
    std::cout << "Object orientation:      " << input->object_pose.pose.orientation.x      << ", " << input->object_pose.pose.orientation.y      << ", " << input->object_pose.pose.orientation.z      << ", " << input->object_pose.pose.orientation.w      << std::endl;
    std::cout << "Transformed orientation: " << transformed.object_pose.pose.orientation.x << ", " << transformed.object_pose.pose.orientation.y << ", " << transformed.object_pose.pose.orientation.z << ", " << transformed.object_pose.pose.orientation.w << std::endl;
    std::cout << std::endl;
  }
  // Try adding transformed object to map
  thesis::DatabaseGetByID db_get_by_type_service;
  db_get_by_type_service.request.id = input->object_id;
  if(db_get_by_type_client.call(db_get_by_type_service))
  {
    // Compute min distance
    // the object needs to have to existing objects of the same type
    // in order to be considered a new object
    float min_distance = (db_get_by_type_service.response.sample.width
                       +  db_get_by_type_service.response.sample.height) / 2;
    // Add object to semantic map
    semantic_map.add(transformed, min_distance);
  }
  else
  {
    // Still add object to map
    semantic_map.add(transformed);
    // Error
    ROS_WARN("Failed to call service 'thesis_database/get_by_type'.");
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
  
  // Get global parameters
  nh.getParam("/thesis/camera_frame", camera_frame);
  nh.getParam("/thesis/map_frame",    map_frame);
  nh.getParam("/thesis/tf_timeout",   tf_timeout);
  
  ROS_INFO("Mapping: ");
  ROS_INFO("  Camera frame: %s.", camera_frame.c_str());
  ROS_INFO("  Map frame:    %s.", map_frame.c_str());
  ROS_INFO("  TF timeout:   %f.", tf_timeout);
  
  // Create transform listener
  transform_listener = new tf::TransformListener();
  // Initialize reusable service clients
  ros::service::waitForService("thesis_database/get_by_type", -1);
  db_get_by_type_client = nh.serviceClient<thesis::DatabaseGetByID>("thesis_database/get_by_type");
  // Subscribe to relevant topics
  ros::Subscriber object_subscriber = nh.subscribe("thesis_recognition/objects", 1, object_callback);
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
