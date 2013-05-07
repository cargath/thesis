/**
 * @author Carsten Könemann
 */

// This is a ROS project
#include <ros/ros.h>

// We are working with OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>

// Local headers
#include <thesis/config.h>
#include <thesis/math3d.h>
#include <thesis/object_recognizer.h>

// We want to subscribe to multiple topics with one callback
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace message_filters;

// We want to subscribe to messages of these types
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
using namespace sensor_msgs;

// We want to call these services
#include <thesis/DatabaseGetAll.h>
#include <thesis/DatabaseSetByID.h>

// We are going to publish messages of this type
#include <thesis/ObjectStamped.h>
#include <tf/transform_datatypes.h>

// Debug image window
static const std::string DEBUG_IMAGE_WINDOW = "Debug Image";

// Recognized-Objects Publisher
ros::Publisher object_publisher;

// Reusable service clients
ros::ServiceClient db_set_by_type_client;

// Sample image database
std::vector<ObjectRecognizer::ProcessedSample> database;
std::map<std::string, ObjectRecognizer::ProcessedSample> database_mapped;
// Object recognizer
ObjectRecognizer object_recognizer;
// Camera model used to convert pixels to camera coordinates
image_geometry::PinholeCameraModel camera_model;

void openni_callback(const Image::ConstPtr& rgb_input,
                     const Image::ConstPtr& depth_input,
                     const CameraInfo::ConstPtr& cam_info_input)
{
  // Convert ROS images to OpenCV images
  cv_bridge::CvImagePtr cv_ptr_bgr8,
                        cv_ptr_depth;
  try
  {
    cv_ptr_bgr8  = cv_bridge::toCvCopy(rgb_input,   image_encodings::BGR8);
    cv_ptr_depth = cv_bridge::toCvCopy(depth_input, image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // Depth values are stored as 32bit float
  cv::Mat1f depth_image(cv_ptr_depth->image);
  // Create a copy to draw stuff on (for debugging purposes)
  cv::Mat debug_img = cv_ptr_bgr8->image.clone();
  // Detect objects
  std::vector<ObjectRecognizer::Finding> findings;
  object_recognizer.detectObjects(cv_ptr_bgr8->image, database, findings, &debug_img);
  // Publish recognized objects
  camera_model.fromCameraInfo(cam_info_input);
  for(size_t i = 0; i < findings.size(); i++)
  {
    // Create new message
    thesis::ObjectStamped msg;
    msg.object_id = findings[i].id;
    // Get object points in camera coordinate space
    std::vector<cv::Point3f> camera_coordinates;
    for(size_t j = 0; j < findings[i].image_points.size(); j++)
    {
      // Check if point is located inside bounds of the image
      if(!(findings[i].image_points[j].x <  cv_ptr_bgr8->image.cols
        && findings[i].image_points[j].y <  cv_ptr_bgr8->image.rows
        && findings[i].image_points[j].x >= 0
        && findings[i].image_points[j].y >= 0))
      {
        continue;
      }
      // Get depth value for current pixel
      float depth = depth_image[(int)findings[i].image_points[j].y][(int)findings[i].image_points[j].x];
      // Check if there is a valid depth value for this pixel
      if(isnan(depth) || depth == 0)
      {
        continue;
      }
      // Project object point to camera space
      cv::Point3f camera_coordinate = camera_model.projectPixelTo3dRay(findings[i].image_points[j]);
      // Use distance from depth image to get actual 3D position in camera space
      camera_coordinate *= depth;
      // Add result to our vector holding successfully transformed points
      camera_coordinates.push_back(camera_coordinate);
    }
    // We need at least 3 corners of an (planar) object to calculate its orientation / surface normal
    if(camera_coordinates.size() >= 3)
    {
      // Calculate 2 vectors of a triangle on the (planar) object
      cv::Point3f a = camera_coordinates[1] - camera_coordinates[0],
                  b = camera_coordinates[2] - camera_coordinates[0];
      // Calculate their lengths
      float ma = mag3f(a),
            mb = mag3f(b);
      // Determine which is width and which is height
      // by looking up which is the longer side of the sample image.
      // Takes a few more comparisons,
      // but works with only 3 of an objects 4 corners.
      thesis::DatabaseSetByID db_set_by_type_service;
      ObjectRecognizer::ProcessedSample sample = database_mapped[findings[i].id];
      if(sample.width < sample.height)
      {
        if(ma < mb)
        {
          db_set_by_type_service.request.sample.width  = ma;
          db_set_by_type_service.request.sample.height = mb;
        }
        else
        {
          db_set_by_type_service.request.sample.width  = mb;
          db_set_by_type_service.request.sample.height = ma;
        }
      }
      else
      {
        if(ma < mb)
        {
          db_set_by_type_service.request.sample.width  = mb;
          db_set_by_type_service.request.sample.height = ma;
        }
        else
        {
          db_set_by_type_service.request.sample.width  = ma;
          db_set_by_type_service.request.sample.height = mb;
        }
      }
      // Train database
      if(!db_set_by_type_client.call(db_set_by_type_service))
      {
        ROS_ERROR("Failed to call service 'thesis_database/set_by_type'.");
        return;
      }
      // Calculate their cross product
      cv::Point3f c = cross3f(a, b);
      // Normalize surface normal
      cv::Point3f n = norm3f(c);
      // Convert direction vector (surface normal) to Euler angles
      cv::Point3f ypr = xyz2ypr(n);
      // Get centroid of the object in camera coordinate space
      cv::Point3f centroid = centroid3f(camera_coordinates);
      // Fill message with our calculations
      msg.object_pose.header.stamp     = ros::Time::now();
      msg.object_pose.header.frame_id  = CAMERA_FRAME;
      msg.object_pose.pose.position.x  = centroid.x;
      msg.object_pose.pose.position.y  = centroid.y;
      msg.object_pose.pose.position.z  = centroid.z;
      msg.object_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(ypr.z, ypr.y, ypr.x);
    }
    else
    {
      // No depth value available
      // Fill object pose with placeholders instead of throwing it away,
      // (camera pose - see below - might still be useful)
      msg.object_pose.header.stamp       = ros::Time(0);
      msg.object_pose.header.frame_id    = MAP_FRAME;
      msg.object_pose.pose.position.x    = std::numeric_limits<float>::quiet_NaN();
      msg.object_pose.pose.position.y    = std::numeric_limits<float>::quiet_NaN();
      msg.object_pose.pose.position.z    = std::numeric_limits<float>::quiet_NaN();
      msg.object_pose.pose.orientation.x = std::numeric_limits<float>::quiet_NaN();
      msg.object_pose.pose.orientation.y = std::numeric_limits<float>::quiet_NaN();
      msg.object_pose.pose.orientation.z = std::numeric_limits<float>::quiet_NaN();
      msg.object_pose.pose.orientation.w = std::numeric_limits<float>::quiet_NaN();
    }
    // Get camera angles
    cv::Point3f ypr_camera = xyz2ypr(cv::Point3f(0.0f, 0.0f, 1.0f));
    // Fill message with our calculations
    msg.camera_pose.header.stamp     = ros::Time::now();
    msg.camera_pose.header.frame_id  = CAMERA_FRAME;
    msg.camera_pose.pose.position.x  = 0;
    msg.camera_pose.pose.position.y  = 0;
    msg.camera_pose.pose.position.z  = 0;
    msg.camera_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(ypr_camera.z, ypr_camera.y, ypr_camera.x);
    // Publish message
    object_publisher.publish(msg);
  }
  // Show debug image
  cv::imshow(DEBUG_IMAGE_WINDOW, debug_img);
  cv::waitKey(3);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "thesis_recognition");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  // Create debug image window
  cv::namedWindow(DEBUG_IMAGE_WINDOW);
  // Create image database
  ros::ServiceClient db_get_all_client = nh.serviceClient<thesis::DatabaseGetAll>("thesis_database/get_all");
  thesis::DatabaseGetAll db_get_all_service;
  if(db_get_all_client.call(db_get_all_service))
  {
    for(size_t i = 0; i < db_get_all_service.response.samples.size(); i++)
    {
      // Convert ROS images to OpenCV images
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(db_get_all_service.response.samples[i].image, image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return 1;
      }
      ROS_INFO("Loading sample '%s'.", db_get_all_service.response.samples[i].id.c_str());
      ObjectRecognizer::ProcessedSample sample = object_recognizer.processSample(cv_ptr->image, db_get_all_service.response.samples[i].id);
      database.push_back(sample);
      database_mapped[db_get_all_service.response.samples[i].id] = sample;
    }
  }
  else
  {
    ROS_ERROR("Failed to call service 'thesis_database/get_all'.");
    return 1;
  }
  // Initialize reusable service clients
  db_set_by_type_client = nh.serviceClient<thesis::DatabaseSetByID>("thesis_database/set_by_type");
  // Enable user to change topics to run the node on different devices
  std::string rgb_topic, depth_topic, cam_info_topic;
  nh.param("rgb_topic", rgb_topic, std::string("camera/rgb/image_rect_color"));
  nh.param("depth_topic", depth_topic, std::string("camera/depth_registered/image_rect"));
  nh.param("cam_info_topic", cam_info_topic, std::string("camera/depth_registered/camera_info"));
  // Subscribe to relevant topics
  Subscriber<Image> rgb_subscriber(nh, rgb_topic, 1);
  Subscriber<Image> depth_subscriber(nh, depth_topic, 1);
  Subscriber<CameraInfo> cam_info_subscriber(nh, cam_info_topic, 1);
  // Use one time-sychronized callback for all subscriptions
  typedef sync_policies::ApproximateTime<Image, Image, CameraInfo> SyncPolicy;
  Synchronizer<SyncPolicy> synchronizer(SyncPolicy(10), rgb_subscriber, depth_subscriber, cam_info_subscriber);
  synchronizer.registerCallback(boost::bind(&openni_callback, _1, _2, _3));
  // Publish recognized objects
  object_publisher = nh_private.advertise<thesis::ObjectStamped>("objects", 1000);
  // Spin
  ros::spin();
  // Free memory
  cv::destroyWindow(DEBUG_IMAGE_WINDOW);
  // Exit
  return 0;
}
