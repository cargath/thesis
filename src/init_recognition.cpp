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
#include <thesis/DatabaseList.h>
#include <thesis/DatabaseGetByID.h>

// We are going to publish messages of this type
#include <thesis/ObjectStamped.h>
#include <tf/transform_datatypes.h>

// Debug image window
static const std::string DEBUG_IMAGE_WINDOW = "Debug Image";

// Recognized-Objects Publisher
ros::Publisher object_publisher;

// Sample image database
std::vector<ObjectRecognizer::ProcessedSample> database;
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
      // Calculate their cross product
      cv::Point3f c = cross3f(a, b);
      // Normalize surface normal
      cv::Point3f n = norm3f(c);
      // Convert direction vector (surface normal) to Euler angles
      cv::Point3f ypr = xyz2ypr(n);
      // Get centroid of the object in camera coordinate space
      cv::Point3f centroid = centroid3f(camera_coordinates);
      // Create a new message object and fill it with our calculations
      thesis::ObjectStamped msg;
      msg.object_id                     = findings[i].id;
      msg.pose_stamped.header.stamp     = ros::Time::now();
      msg.pose_stamped.header.frame_id  = CAMERA_FRAME;
      msg.pose_stamped.pose.position.x  = centroid.x;
      msg.pose_stamped.pose.position.y  = centroid.y;
      msg.pose_stamped.pose.position.z  = centroid.z;
      msg.pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(ypr.z, ypr.y, ypr.x);
      // Publish message
      object_publisher.publish(msg);
    }
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
  // Create debug image window
  cv::namedWindow(DEBUG_IMAGE_WINDOW);
  // Create image database
  ros::ServiceClient db_list_client = nh.serviceClient<thesis::DatabaseList>("thesis/database/list");
  thesis::DatabaseList list_srv;
  if(db_list_client.call(list_srv))
  {
    ros::ServiceClient db_get_client = nh.serviceClient<thesis::DatabaseGetByID>("thesis/database/getByID");
    thesis::DatabaseGetByID get_srv;
    for(size_t i = 0; i < list_srv.response.list.size(); i++)
    {
      get_srv.request.id = list_srv.response.list[i];
      if(db_get_client.call(get_srv))
      {
        // Convert ROS images to OpenCV images
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(get_srv.response.sample.image, image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return 1;
        }
        ROS_INFO("Loading sample '%s'.", get_srv.response.sample.id.c_str());
        ObjectRecognizer::ProcessedSample sample = object_recognizer.processSample(cv_ptr->image, get_srv.response.sample.id);
        database.push_back(sample);
      }
      else
      {
        ROS_ERROR("Failed to call service 'thesis/database/getByID'.");
        return 1;
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to call service 'thesis/database/list'.");
    return 1;
  }
  // Subscribe to relevant topics
  Subscriber<Image> rgb_subscriber(nh, "camera/rgb/image_rect_color", 1);
  Subscriber<Image> depth_subscriber(nh, "camera/depth_registered/image_rect", 1);
  Subscriber<CameraInfo> cam_info_subscriber(nh, "camera/depth_registered/camera_info", 1);
  typedef sync_policies::ApproximateTime<Image, Image, CameraInfo> SyncPolicy;
  Synchronizer<SyncPolicy> synchronizer(SyncPolicy(10), rgb_subscriber, depth_subscriber, cam_info_subscriber);
  synchronizer.registerCallback(boost::bind(&openni_callback, _1, _2, _3));
  // Publish recognized objects
  object_publisher = nh.advertise<thesis::ObjectStamped>("thesis/recognition/objects", 1000);
  // Spin
  ros::spin();
  // Free memory
  cv::destroyWindow(DEBUG_IMAGE_WINDOW);
  // Exit
  return 0;
}
