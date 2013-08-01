/**
 * @author Carsten Könemann
 */

// This is a ROS project
#include <ros/ros.h>

// Local headers
#include <thesis/math3d.h>

// We are working with TF
#include <tf/transform_listener.h>

// We want to call these services
#include <thesis/DatabaseGetByID.h>
#include <thesis/MappingGetAll.h>

// We are going to publish messages of these types
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Config parameters
std::string map_frame;

bool        debug;

// Reusable service clients
ros::ServiceClient db_get_by_type_client;
ros::ServiceClient map_get_all_client;

// Reusable publishers
ros::Publisher object_pose_publisher;

// Store markers globally (in order to remove them before publishing new ones)
visualization_msgs::MarkerArray markers;
unsigned int unique_marker_id;

void clear_markers()
{
  // Remove markers
  for(size_t i = 0; i < markers.markers.size(); i++)
  {
    markers.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  object_pose_publisher.publish(markers);
  // Clear array
  markers.markers.clear();
  // Reset marker IDs
  unique_marker_id = 0;
}

void create_markers(thesis::ObjectInstance object)
{
  visualization_msgs::Marker object_marker;
  
  if(debug)
  {
    ROS_INFO("Quaternion before normalization: (%f, %f, %f, %f).",
             object.pose_stamped.pose.orientation.x,
             object.pose_stamped.pose.orientation.y,
             object.pose_stamped.pose.orientation.z,
             object.pose_stamped.pose.orientation.w);
  }
  
  normalize_quaternion_msg(object.pose_stamped.pose.orientation);
  
  if(debug)
  {
    ROS_INFO("Quaternion after normalization:  (%f, %f, %f, %f).",
             object.pose_stamped.pose.orientation.x,
             object.pose_stamped.pose.orientation.y,
             object.pose_stamped.pose.orientation.z,
             object.pose_stamped.pose.orientation.w);
    std::cout << std::endl;
  }
  
  //
  if(isnan(object.pose_stamped.pose.orientation.w))
  {
    tf::quaternionTFToMsg(IDENTITY_QUATERNION, object.pose_stamped.pose.orientation);
  }

  // Setup the values that are valid for all markers
  object_marker.ns              = "thesis_example";
  object_marker.action          = visualization_msgs::Marker::ADD;
  object_marker.header.frame_id = map_frame;
  object_marker.header.stamp    = ros::Time();
  object_marker.color.a         = 1.0;
  
  // Arrow; x-axis
  object_marker.id      = unique_marker_id++;
  object_marker.type    = visualization_msgs::Marker::ARROW;
  object_marker.pose    = object.pose_stamped.pose;
  object_marker.scale.x = 0.2;
  object_marker.scale.y = 0.02;
  object_marker.scale.z = 0.02;
  object_marker.color.r = 1.0;
  object_marker.color.g = 0.0;
  object_marker.color.b = 0.0;
  markers.markers.push_back(object_marker);
  
  // Rotate object pose to represent the other axis
  tf::Quaternion q;
  tf::quaternionMsgToTF(object.pose_stamped.pose.orientation, q);
  tf::Matrix3x3 matTemp(q);
  double y,
         p,
         r;
  matTemp.getRPY(r, p, y);
  double y_ = y + M_PI / 2,
         p_ = p - M_PI / 2;
  
  // Arrow; y-axis
  object_marker.id               = unique_marker_id++;
  object_marker.color.r          = 0.0;
  object_marker.color.g          = 1.0;
  object_marker.color.b          = 0.0;
  object_marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, y_);
  markers.markers.push_back(object_marker);
  
  // Arrow; z-axis
  object_marker.id               = unique_marker_id++;
  object_marker.color.r          = 0.0;
  object_marker.color.g          = 0.0;
  object_marker.color.b          = 1.0;
  object_marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p_, y);
  markers.markers.push_back(object_marker);
  
  //
  thesis::DatabaseGetByID db_get_by_type_service;
  db_get_by_type_service.request.id = object.type_id;
  if(db_get_by_type_client.call(db_get_by_type_service))
  {
    double w = db_get_by_type_service.response.object_class.width,
           h = db_get_by_type_service.response.object_class.height;
    //
    if(isnan(w) || (w == 0.0))
    {
      w = 0.01;
    }
    if(isnan(h) || (h == 0.0))
    {
      h = 0.01;
    }
    // Box; representing a 3D render of the object
    object_marker.id      = unique_marker_id++;
    object_marker.type    = visualization_msgs::Marker::CUBE;
    object_marker.pose    = object.pose_stamped.pose;
    object_marker.scale.x = w;
    object_marker.scale.y = h;
    object_marker.scale.z = 0.01;
    object_marker.color.r = 0.0;
    object_marker.color.g = 1.0;
    object_marker.color.b = 0.0;
    markers.markers.push_back(object_marker);
  }
  else
  {
    ROS_WARN("Visualisation: ");
    ROS_WARN("  Failed to call service 'thesis_database/get_by_type'.");
    ROS_WARN("  Visualize only axes, not 3D render of the object.");
  }
  
  // Text; identifies the objects type (since we can't use textures)
  object_marker.id      = unique_marker_id++;
  object_marker.type    = visualization_msgs::Marker::TEXT_VIEW_FACING;
  object_marker.pose    = object.pose_stamped.pose;
  object_marker.scale.z = 0.1;
  object_marker.color.r = 1.0;
  object_marker.color.g = 1.0;
  object_marker.color.b = 1.0;
  object_marker.text    = object.type_id;
  markers.markers.push_back(object_marker);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "thesis_example");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  // Get global parameters
  nh.getParam("/thesis/map_frame", map_frame);
  
  ROS_INFO("Visualisation (global parameters): ");
  ROS_INFO("  Map frame: %s.", map_frame.c_str());
  std::cout << std::endl;
  
  // Get local parameters
  nh_private.param("debug", debug, false);
  
  ROS_INFO("Visualisation (local parameters): ");
  ROS_INFO("  Debug mode: %s.", debug ? "true" : "false");
  std::cout << std::endl;
  
  // Initialize reusable service clients
  ros::service::waitForService("thesis_database/get_by_type", -1);
  db_get_by_type_client = nh.serviceClient<thesis::DatabaseGetByID>("thesis_database/get_by_type");
  ros::service::waitForService("thesis_mapping/all", -1);
  map_get_all_client = nh.serviceClient<thesis::MappingGetAll>("thesis_mapping/all");
  // Publish transformed object + camera poses for debug purposes
  object_pose_publisher = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  // Clear, create new and publish markers
  ros::Duration wait_duration(1);
  while(ros::ok())
  {
    // Remove previously published markers
    clear_markers();
    // Get objects from semantic map
    thesis::MappingGetAll map_get_all_service;
    if(map_get_all_client.call(map_get_all_service))
    {
      // Add identity pose
      // (for verifying axis-arrow markers)
      thesis::ObjectInstance msg;
      msg.type_id = "Identity";
      msg.pose_stamped.header.stamp       = ros::Time::now();
      msg.pose_stamped.header.frame_id    = map_frame;
      msg.pose_stamped.pose.position.x    = 0;
      msg.pose_stamped.pose.position.y    = 0;
      msg.pose_stamped.pose.position.z    = 0;
      tf::quaternionTFToMsg(IDENTITY_QUATERNION, msg.pose_stamped.pose.orientation);
      map_get_all_service.response.objects.push_back(msg);
      // Create various markers for every object
      for(size_t i = 0; i < map_get_all_service.response.objects.size(); i++)
      {
        if(debug)
        {
          ROS_INFO("Creating markers for #%lo: ", i);
          ROS_INFO("%s", map_get_all_service.response.objects[i].type_id.c_str());
        }
        create_markers(map_get_all_service.response.objects[i]);
      }
    }
    else
    {
      ROS_WARN("Visualisation: ");
      ROS_WARN("  Failed to call service 'thesis_mapping/all'.");
      ROS_WARN("  Unable to visualize the semantic map.");
    }
    // Publish visualization markers
    object_pose_publisher.publish(markers);
    // Spin
    ros::spinOnce();
    // Don't publish markers in real-time ;)
    wait_duration.sleep();
  }
  // Exit
  return 0;
}
