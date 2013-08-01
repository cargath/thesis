/**
 * @author Carsten Könemann
 */

// This is a ROS project
#include <ros/ros.h>

// Local headers
#include <thesis/math3d.h>
#include <thesis/semantic_map.h>

// We are working with TF
#include <tf/transform_listener.h>

// We want to subscribe to messages of this types
#include <thesis/ObjectInstance.h>

// We want to call these services
#include <thesis/DatabaseGetByID.h>

// This node provides these services
#include <thesis/MappingGetAll.h>
#include <thesis/MappingGetByID.h>
#include <thesis/MappingGetByPosition.h>

using namespace geometry_msgs;

// Config parameters
std::string camera_frame,
            map_frame;

double      tf_timeout;

int         memory_size;

bool        debug;

// Transform listener
tf::TransformListener* transform_listener;

// Reusable service clients
ros::ServiceClient db_get_by_type_client;

// The actual semantic map, storing recognized objects with map coordinates
SemanticMap semantic_map;

void camera_2_map(const PoseStamped& pose_camera, PoseStamped& pose_map)
{
  //
  transform_listener->waitForTransform(
    camera_frame,
    map_frame,
    pose_camera.header.stamp,
    ros::Duration(tf_timeout)
  );
  // Transform recognized camera pose to map frame
  transform_listener->transformPose(map_frame, pose_camera, pose_map);
  pose_map.header.stamp = ros::Time(0);
}

cv::Point3f get_current_camera_position()
{
  //
  geometry_msgs::PoseStamped pose_camera;
  pose_camera.header.stamp    = ros::Time::now();
  pose_camera.header.frame_id = camera_frame;
  pose_camera.pose.position.x = 0;
  pose_camera.pose.position.y = 0;
  pose_camera.pose.position.z = 0;
  tf::quaternionTFToMsg(IDENTITY_QUATERNION, pose_camera.pose.orientation);
  //
  geometry_msgs::PoseStamped pose_map;
  camera_2_map(pose_camera, pose_map);
  //
  cv::Point3f p;
  p.x = pose_map.pose.position.x;
  p.y = pose_map.pose.position.y;
  p.z = pose_map.pose.position.z;
  //
  return p;
}

void object_callback(const thesis::ObjectInstance::ConstPtr& input)
{
  // 
  thesis::ObjectInstance transformed;
  transformed.type_id = input->type_id;
  // If available, transform recognized object pose to map frame
  if(!isnan(input->pose_stamped.pose.position))
  {
    // Transform object to map space
    camera_2_map(input->pose_stamped, transformed.pose_stamped);
    
    // Debug information
    if(debug)
    {
      ROS_INFO("Mapping: Object caught: %s.", input->type_id.c_str());
      
      ROS_INFO("  Object      position:    (%f, %f, %f).",
               input->pose_stamped.pose.position.x,
               input->pose_stamped.pose.position.y,
               input->pose_stamped.pose.position.z);
               
      ROS_INFO("  Object      orientation: (%f, %f, %f, %f).",
               input->pose_stamped.pose.orientation.x,
               input->pose_stamped.pose.orientation.y,
               input->pose_stamped.pose.orientation.z,
               input->pose_stamped.pose.orientation.w);
      
      ROS_INFO("  Transformed position:    (%f, %f, %f).",
               transformed.pose_stamped.pose.position.x,
               transformed.pose_stamped.pose.position.y,
               transformed.pose_stamped.pose.position.z);
      
      ROS_INFO("  Transformed orientation: (%f, %f, %f, %f).",
               transformed.pose_stamped.pose.orientation.x,
               transformed.pose_stamped.pose.orientation.y,
               transformed.pose_stamped.pose.orientation.z,
               transformed.pose_stamped.pose.orientation.w);

      std::cout << std::endl;
    }
  }
  // Try adding transformed object to map
  thesis::DatabaseGetByID db_get_by_type_service;
  db_get_by_type_service.request.id = input->type_id;
  if(db_get_by_type_client.call(db_get_by_type_service))
  {
    // Compute min distance
    // the object needs to have to existing objects of the same type
    // in order to be considered a new object
    float min_distance = (db_get_by_type_service.response.object_class.width
                       +  db_get_by_type_service.response.object_class.height) / 2;
    // Add object to semantic map
    semantic_map.add(transformed, min_distance);
  }
  else
  {
    // Still add object to map
    semantic_map.add(transformed);
    // Error
    ROS_WARN("Mapping: Failed to call service 'thesis_database/get_by_type'.");
    return;
  }
}

bool get_all(thesis::MappingGetAll::Request& request,
             thesis::MappingGetAll::Response& result)
{
  semantic_map.setCurrentPosition(get_current_camera_position());
  semantic_map.getAll(result.objects);
  return true;
}

bool get_by_type(thesis::MappingGetByID::Request& request,
                 thesis::MappingGetByID::Response& result)
{
  semantic_map.setCurrentPosition(get_current_camera_position());
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
  semantic_map.setCurrentPosition(get_current_camera_position());
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
  nh.getParam("/thesis/memory_size",  memory_size);
  
  ROS_INFO("Mapping (global parameters): ");
  ROS_INFO("  Camera frame: %s.", camera_frame.c_str());
  ROS_INFO("  Map frame:    %s.", map_frame.c_str());
  ROS_INFO("  TF timeout:   %f.", tf_timeout);
  ROS_INFO("  Memory size:  %i.", memory_size);
  std::cout << std::endl;
  
  //
  semantic_map = SemanticMap(memory_size);
  
  // Get local parameters
  nh_private.param("debug", debug, false);
  
  ROS_INFO("Mapping (local parameters): ");
  ROS_INFO("  Debug mode: %s.", debug ? "true" : "false");
  std::cout << std::endl;
  
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
