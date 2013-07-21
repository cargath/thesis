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
#include <thesis/math2d.h>
#include <thesis/math3d.h>
#include <thesis/object_recognizer.h>

// We want to subscribe to multiple topics with one callback
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace message_filters;

// We want to subscribe to messages of these types
#include <std_msgs/Empty.h>
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

// FPS counter for debugging purposes
#include <thesis/fps_calculator.h>

// Debug image window
#define CAMERA_DEBUG_IMAGE_WINDOW "Camera Debug Image"
#define MIPMAP_DEBUG_IMAGE_WINDOW "Mipmap Debug Image"

// Config parameters
std::string camera_frame,
            map_frame;
            
int         mipmap_level,
            max_objects_per_frame,
            max_nof_keypoints;

// Camera orientation seen from the camera POV
static const cv::Point3f YPR_CAMERA = xyz2ypr(cv::Point3f(0.0f, 0.0f, 1.0f));

// FPS counter for debugging purposes
FPSCalculator fps_calculator;

// Recognized-Objects Publisher
ros::Publisher object_publisher;

// Reusable service clients
ros::ServiceClient db_get_all_client,
                   db_set_by_type_client;

// Multiple points
typedef std::vector<cv::Point2f> Cluster2f;
// The corner points of an object in the camera image and the ID of this object
typedef std::pair<std::string, Cluster2f> IDClusterPair;
// Map multiple points to the corresponding ID
typedef std::map<std::string, Cluster2f> Points2IDMap;

// A pair of ImageInfo (here used for a sample image and its mipmap)
typedef std::pair<ObjectRecognizer::ImageInfo, ObjectRecognizer::ImageInfo> ImageInfoPair;
// Map a sample image and its mipmap to the corresponding ID
typedef std::map<std::string, ImageInfoPair> ProcessedDatabase;

// The processed database of sample images
ProcessedDatabase database_processed;

// Object recognizer
ObjectRecognizer object_recognizer;

// Camera model used to convert pixels to camera coordinates
image_geometry::PinholeCameraModel camera_model;

// Remember when we have successfully recognized an object and where
std::vector<IDClusterPair> tracking_objects;

inline void create_mipmap(const cv::Mat& image, cv::Mat& mipmap, unsigned int n)
{
  mipmap = image.clone();
  for(unsigned int i = 0; i < n; i++)
  {
    cv::pyrDown(mipmap, mipmap);
  }
}

bool reset(int mipmaps)
{
  // Set mipmap level
  if(mipmaps < 0)
  {
    ROS_ERROR("Mipmap level (%i) out of bounds.", mipmaps);
    return false;
  }
  ROS_INFO("Mipmap level: %i.", mipmaps);
  mipmap_level = mipmaps;
  // Create image database
  ROS_INFO("Processing database...");
  database_processed.clear();
  thesis::DatabaseGetAll db_get_all_service;
  if(db_get_all_client.call(db_get_all_service))
  {
    for(size_t i = 0; i < db_get_all_service.response.samples.size(); i++)
    {
      // Convert ROS images to OpenCV images
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(db_get_all_service.response.samples[i].image, image_encodings::MONO8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
      }
      ROS_INFO("Loading sample '%s'.", db_get_all_service.response.samples[i].id.c_str());
      // Process sample image (compute keypoints and descriptors)
      object_recognizer.setMaxKeypoints(max_nof_keypoints);
      ObjectRecognizer::ImageInfo image_info;
      object_recognizer.getImageInfo(cv_ptr->image, image_info);
      database_processed[db_get_all_service.response.samples[i].id].first = image_info;
      // Create mipmap image
      cv::Mat sample_mipmap;
      create_mipmap(cv_ptr->image, sample_mipmap, mipmap_level);
      // Process mipmap image (compute keypoints and descriptors)
      object_recognizer.setMaxKeypoints(max_nof_keypoints / (mipmap_level+1));
      ObjectRecognizer::ImageInfo mipmap_info;
      object_recognizer.getImageInfo(sample_mipmap, mipmap_info);
      database_processed[db_get_all_service.response.samples[i].id].second = mipmap_info;
    }
    ROS_INFO("...done.");
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call service 'thesis_database/get_all'.");
    return false;
  }
}

inline bool call_db_set_by_type(std::string id, float width, float height)
{
  thesis::DatabaseSetByID db_set_by_type_service;
  db_set_by_type_service.request.sample.id     = id;
  db_set_by_type_service.request.sample.width  = width;
  db_set_by_type_service.request.sample.height = height;
  if(db_set_by_type_client.call(db_set_by_type_service))
  {
    return true;
  }
  else
  {
    return false;
  }
}

inline geometry_msgs::Quaternion quaternion_from_plane(cv::Point3f w, cv::Point3f h)
{
  ROS_INFO("w:   (%f, %f, %f)", w.x, w.y, w.z);
  ROS_INFO("h:   (%f, %f, %f)", h.x, h.y, h.z);
  // Calculate the cross product of the given sides of the plane
  cv::Point3f c = cross3f(w, h);
  ROS_INFO("c:   (%f, %f, %f)", c.x, c.y, c.z);
  // Normalize surface normal
  cv::Point3f n = norm3f(c);
  ROS_INFO("n:   (%f, %f, %f)", n.x, n.y, n.z);
  // If the camera successfully recognizes an object,
  // the object obviously faces the camera
  n.z = fabs(n.z);
  ROS_INFO("abs: (%f, %f, %f)", n.x, n.y, n.z);
  // Convert direction vector (surface normal) to Euler angles
  cv::Point3f ypr = xyz2ypr(n);
  ROS_INFO("ypr: (%f, %f, %f)", ypr.x, ypr.y, ypr.z);
  // Convert Euler angles to quaternion
  return tf::createQuaternionMsgFromRollPitchYaw(ypr.z, ypr.y, ypr.x);
}

inline bool draw_rectangle(const std::vector<cv::Point2f>& corners,
                           const cv::Scalar& color,
                           cv::Mat& debug_image)
{
  if(corners.size() == 4)
  {
    line(debug_image, corners[0], corners[1], color);
    line(debug_image, corners[1], corners[2], color);
    line(debug_image, corners[2], corners[3], color);
    line(debug_image, corners[3], corners[0], color);
    return true;
  }
  else
  {
    return false;
  }
}

inline float get_depth(const cv::Mat1f& depth_image, const cv::Point2f& point)
{
  if(point.x < depth_image.cols && point.y < depth_image.rows
     && point.x >= 0 && point.y >= 0)
  {
    float depth = depth_image[(int)point.y][(int)point.x];
    return depth;
  }
  else
  {
    return NAN;
  }
}

inline void publish_object(const IDClusterPair& finding,
                           const cv::Mat& mono8_image,
                           const cv::Mat1f& depth_image)
{
  // Create new message
  thesis::ObjectStamped msg;
  msg.object_id = finding.first;
  msg.object_pose.header.stamp     = ros::Time::now();
  msg.object_pose.header.frame_id  = camera_frame;
  msg.camera_pose.header.stamp     = msg.object_pose.header.stamp;
  msg.camera_pose.header.frame_id  = camera_frame;
  msg.camera_pose.pose.position.x  = 0;
  msg.camera_pose.pose.position.y  = 0;
  msg.camera_pose.pose.position.z  = 0;
  msg.camera_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(YPR_CAMERA.z, YPR_CAMERA.y, YPR_CAMERA.x);
  // Get object points in camera coordinate space
  std::vector<cv::Point3f> camera_coordinates,
                           good_camera_coordinates;
  for(size_t i = 0; i < finding.second.size(); i++)
  {
    // Get depth value for current pixel
    float depth = get_depth(depth_image, finding.second[i]);
    // Check if there is a valid depth value for this pixel
    if(isnan(depth) || depth == 0)
    {
      camera_coordinates.push_back(cv::Point3f(NAN, NAN, NAN));
      continue;
    }
    // Project object point to camera coordinate space
    cv::Point3f camera_coordinate = camera_model.projectPixelTo3dRay(finding.second[i]);
    // Use distance from depth image to get actual 3D position in camera space
    camera_coordinate *= depth;
    // Add result to our vector holding successfully transformed points
    camera_coordinates.push_back(camera_coordinate);
    good_camera_coordinates.push_back(camera_coordinate);
  }
  // Get object position in camera coordinate space
  if(good_camera_coordinates.size() >= 1)
  {
    cv::Point3f centroid = centroid3f(good_camera_coordinates);
    msg.object_pose.pose.position.x = centroid.x;
    msg.object_pose.pose.position.y = centroid.y;
    msg.object_pose.pose.position.z = centroid.z;
  }
  else
  {
    // No depth value available.
    // Fill object pose with placeholders instead of throwing it away
    // (camera pose - see below - might still be useful)
    msg.object_pose.header.stamp    = ros::Time(0);
    msg.object_pose.header.frame_id = map_frame;
    msg.object_pose.pose.position.x = NAN;
    msg.object_pose.pose.position.y = NAN;
    msg.object_pose.pose.position.z = NAN;
  }
  // Calculate 2 vectors of a triangle on the (planar) object
  cv::Point3f w, h;
  if(!isnan(camera_coordinates[0]) && !isnan(camera_coordinates[1]) && !isnan(camera_coordinates[3]))
  {
    w = camera_coordinates[1] - camera_coordinates[0],
    h = camera_coordinates[3] - camera_coordinates[0];
  }
  else if(!isnan(camera_coordinates[1]) && !isnan(camera_coordinates[2]) && !isnan(camera_coordinates[0]))
  {
    h = camera_coordinates[2] - camera_coordinates[1],
    w = camera_coordinates[0] - camera_coordinates[1];
  }
  else if(!isnan(camera_coordinates[2]) && !isnan(camera_coordinates[3]) && !isnan(camera_coordinates[1]))
  {
    w = camera_coordinates[3] - camera_coordinates[2],
    h = camera_coordinates[1] - camera_coordinates[2];
  }
  else if(!isnan(camera_coordinates[3]) && !isnan(camera_coordinates[0]) && !isnan(camera_coordinates[2]))
  {
    h = camera_coordinates[0] - camera_coordinates[3],
    w = camera_coordinates[2] - camera_coordinates[3];
  }
  else
  {
    w = cv::Point3f(NAN, NAN, NAN);
    h = cv::Point3f(NAN, NAN, NAN);
  }
  // Get object orientation & dimensions in camera coordinate space
  if(!(isnan(w) || isnan(h)))
  {
    ROS_INFO("Perception: !(isnan(w) || isnan(h)) == true");
    // Update database with their dimensions
    if(!call_db_set_by_type(finding.first, mag3f(w), mag3f(h)))
    {
      ROS_ERROR("Failed to call service 'thesis_database/set_by_type'.");
    }
    // Compute object orientation
    msg.object_pose.pose.orientation = quaternion_from_plane(w, h);
  }
  else
  {
    ROS_INFO("Perception: !(isnan(w) || isnan(h)) == false");
    msg.object_pose.pose.orientation.x = NAN;
    msg.object_pose.pose.orientation.y = NAN;
    msg.object_pose.pose.orientation.z = NAN;
    msg.object_pose.pose.orientation.w = NAN;
  }
  // Publish message
  object_publisher.publish(msg);
}

inline void recognize(const std::string& sample_id,
                      ObjectRecognizer::ImageInfo& sample_image_info,
                      const cv::Mat& camera_image,
                      ObjectRecognizer::ImageInfo& camera_image_info,
                      const unsigned int max_loops,
                      std::vector<IDClusterPair>& findings,
                      cv::Mat& debug_image)
{
  Cluster2f object_points;
  // Search for an object until we don't find any more occurrences
  for(unsigned int loops = 0; loops <= max_loops; loops++)
  {
    if(object_recognizer.recognize(sample_image_info, camera_image_info, object_points))
    {
      // Remove keypoints belonging to this candidate from image info,
      // in order to search for other candidates of the same type
      ObjectRecognizer::ImageInfo inside_mask;
      object_recognizer.filterImageInfo(camera_image_info, object_points, &inside_mask, &camera_image_info);
      // 
      if(inside_mask.keypoints.size() >= 4)
      {
        // Visualize result
        draw_rectangle(object_points, GREEN, debug_image);
        // We want to search for this candidate in the original camera image for higher precision
        findings.push_back(IDClusterPair(sample_id, object_points));
        cv::drawKeypoints(debug_image, inside_mask.keypoints, debug_image, YELLOW);
        // Start next search with an empty vector again
        object_points.clear();
      }
      else
      {
        // Draw result one last time in order to visualize false positives
        draw_rectangle(object_points, RED, debug_image);
        // We won't find any more occurrences
        break;
      }
    }
    else
    {
      // Draw result one last time in order to visualize false positives
      draw_rectangle(object_points, RED, debug_image);
      // We won't find any more occurrences
      break;
    }
  }
}

void callback_simple(const cv::Mat& camera_image,
                     cv::Mat& camera_debug_image,
                     std::vector<IDClusterPair>& findings)
{
  // Process camera image
  object_recognizer.setMaxKeypoints(max_nof_keypoints);
  ObjectRecognizer::ImageInfo cam_img_info;
  object_recognizer.getImageInfo(camera_image, cam_img_info);
  // Draw keypoints to debug image
  cv::drawKeypoints(camera_debug_image, cam_img_info.keypoints, camera_debug_image, YELLOW);
  // Recognize objects
  for(ProcessedDatabase::iterator it = database_processed.begin(); it != database_processed.end(); it++)
  {
    recognize(it->first,
              it->second.first,
              camera_image,
              cam_img_info,
              max_objects_per_frame,
              findings,
              camera_debug_image);
  }
  // Show debug image
  cv::imshow(CAMERA_DEBUG_IMAGE_WINDOW, camera_debug_image);
  cv::waitKey(3);
}

void callback_mipmapping(const cv::Mat& camera_image,
                         cv::Mat& camera_debug_image,
                         std::vector<IDClusterPair>& findings)
{
  // Create mipmap of camera image
  cv::Mat cam_img_mipmap;
  create_mipmap(camera_image, cam_img_mipmap, mipmap_level);
  // Create copy to draw stuff on (for debugging purposes)
  cv::Mat mipmap_debug_image;
  create_mipmap(camera_debug_image, mipmap_debug_image, mipmap_level);
  // Process the mipmap of the camera image
  object_recognizer.setMaxKeypoints(max_nof_keypoints / (mipmap_level+1));
  ObjectRecognizer::ImageInfo cam_img_mipmap_info;
  object_recognizer.getImageInfo(cam_img_mipmap, cam_img_mipmap_info);
  // Draw keypoints to debug image
  cv::drawKeypoints(mipmap_debug_image, cam_img_mipmap_info.keypoints, mipmap_debug_image, BLUE);
  // Recognize sample mipmaps on camera mipmap
  std::vector<IDClusterPair> mipmap_findings;
  for(ProcessedDatabase::iterator it = database_processed.begin(); it != database_processed.end(); it++)
  {
    recognize(it->first,
              it->second.second,
              camera_image,
              cam_img_mipmap_info,
              max_objects_per_frame,
              mipmap_findings,
              mipmap_debug_image);
  }
  // We did not find anything on the mipmap camera image,
  // and we didn't find anything last time.
  // Don't look any further.
  if(mipmap_findings.size() < 1 && tracking_objects.size() < 1)
  {
    // Show debug image
    cv::imshow(CAMERA_DEBUG_IMAGE_WINDOW, camera_debug_image);
    cv::imshow(MIPMAP_DEBUG_IMAGE_WINDOW, mipmap_debug_image);
    cv::waitKey(3);
    return;
  }
  // Create a vector holding
  // - all objects found in mipmap camera image
  //   (they are added first, making perception more stable during movement)
  // - all objects successfully found last frame
  // They are the candidates we want to search in the original camera image
  std::vector<IDClusterPair> temp_tracking;
  double scale = pow(2, mipmap_level);
  for(size_t i = 0; i < mipmap_findings.size(); i++)
  {
    // Scale points found on the mipmap image back to original size
    mipmap_findings[i].second[0] *= scale,
    mipmap_findings[i].second[1] *= scale,
    mipmap_findings[i].second[2] *= scale,
    mipmap_findings[i].second[3] *= scale;
    temp_tracking.push_back(mipmap_findings[i]);
  }
  temp_tracking.insert(temp_tracking.end(), tracking_objects.begin(), tracking_objects.end());
  // Clear tracking vector,
  // so we can fill it with objects recognized this frame again
  tracking_objects.clear();
  // Compute keypoints and descriptors for the original camera image
  object_recognizer.setMaxKeypoints(max_nof_keypoints);
  ObjectRecognizer::ImageInfo camera_image_info;
  object_recognizer.getImageInfo(camera_image, camera_image_info);
  cv::drawKeypoints(camera_debug_image, camera_image_info.keypoints, camera_debug_image, BLUE);
  // Recognize objects
  for(std::vector<IDClusterPair>::iterator it = temp_tracking.begin(); it != temp_tracking.end(); it++)
  {
    Cluster2f object_points;
    // Draw the region we are going to examine further
    draw_rectangle(it->second, YELLOW, camera_debug_image);
    // Process only the part of the camera image
    // belonging to the area defined by the points we found on the mipmap image
    ObjectRecognizer::ImageInfo inside_mask,
                                outside_mask;
    object_recognizer.filterImageInfo(camera_image_info,
                                      it->second,
                                      &inside_mask,
                                      &outside_mask);
    // Draw the keypoints belonging to the region we are going to examine
    cv::drawKeypoints(camera_debug_image, inside_mask.keypoints, camera_debug_image, YELLOW);
    // Apply recognizer to the previously processed region of the camera image
    if(object_recognizer.recognize(database_processed[it->first].first, inside_mask, object_points))
    {
      // Visualize result
      draw_rectangle(object_points, GREEN, camera_debug_image);
      // Do not check these keypoints / descriptors anymore,
      // they belong to an already successfully recognized object
      camera_image_info.keypoints   = outside_mask.keypoints;
      camera_image_info.descriptors = outside_mask.descriptors;
      // We are going to publish this finding later
      findings.push_back(IDClusterPair(it->first, object_points));
      // Also remember finding as a candidate for next frame
      tracking_objects.push_back(IDClusterPair(it->first, object_points));
    }
    else
    {
      // Visualize result for unsuccessful passes in a different color
      draw_rectangle(object_points, RED, camera_debug_image);
    }
  }
  // Show debug image
  cv::imshow(CAMERA_DEBUG_IMAGE_WINDOW, camera_debug_image);
  cv::imshow(MIPMAP_DEBUG_IMAGE_WINDOW, mipmap_debug_image);
  cv::waitKey(3);
}

void callback_openni(const Image::ConstPtr& rgb_input,
                     const Image::ConstPtr& depth_input,
                     const CameraInfo::ConstPtr& cam_info_input)
{
  // Update FPS calculator
  fps_calculator.update();
  // Convert ROS image messages to OpenCV images
  cv_bridge::CvImagePtr cv_ptr_bgr8,
                        cv_ptr_mono8,
                        cv_ptr_depth;
  try
  {
    cv_ptr_bgr8  = cv_bridge::toCvCopy(rgb_input,   image_encodings::BGR8);
    cv_ptr_mono8 = cv_bridge::toCvCopy(rgb_input,   image_encodings::MONO8);
    cv_ptr_depth = cv_bridge::toCvCopy(depth_input, image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // Depth values are stored as 32bit float
  cv::Mat1f depth_image = cv::Mat1f(cv_ptr_depth->image);
  //
  std::stringstream stream;
  stream << "FPS: " << fps_calculator.get_fps();
  std::string fps_string = stream.str();
  int font_face = cv::FONT_HERSHEY_PLAIN;
  double font_scale = 2;
  int font_thickness = 2;
  int baseline = 0;
  cv::Size text_size = cv::getTextSize(fps_string, font_face, font_scale, font_thickness, &baseline);
  baseline += font_thickness;
  cv::Point text_org(10, text_size.height + 10);
  cv::Point text_shadow_org(text_org.x + 1, text_org.y + 1);
  cv::putText(cv_ptr_bgr8->image, fps_string, text_shadow_org, font_face, font_scale, BLACK, font_thickness, 8);
  cv::putText(cv_ptr_bgr8->image, fps_string, text_org,        font_face, font_scale, WHITE, font_thickness, 8);
  //
  std::vector<IDClusterPair> findings;
  if(mipmap_level > 0)
  {
    callback_mipmapping(cv_ptr_mono8->image, cv_ptr_bgr8->image, findings);
  }
  else
  {
    callback_simple(cv_ptr_mono8->image, cv_ptr_bgr8->image, findings);
  }
  // Publish objects
  camera_model.fromCameraInfo(cam_info_input);
  for(size_t i = 0; i < findings.size(); i++)
  {
    //publish_object(findings[i], cv_ptr_mono8->image, depth_image);
  }
}

void callback_database_update(const std_msgs::Empty::ConstPtr& input)
{
  // Rebuild the database
  reset(mipmap_level);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "thesis_recognition");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  // Create debug image window
  cv::namedWindow(CAMERA_DEBUG_IMAGE_WINDOW);
  cv::namedWindow(MIPMAP_DEBUG_IMAGE_WINDOW);
  
  // Get global parameters
  std::string rgb_image_topic,
              depth_image_topic,
              camera_info_topic;
  
  nh.getParam("/thesis/rgb_image_topic",   rgb_image_topic);
  nh.getParam("/thesis/depth_image_topic", depth_image_topic);
  nh.getParam("/thesis/camera_info_topic", camera_info_topic);
  nh.getParam("/thesis/camera_frame",      camera_frame);
  nh.getParam("/thesis/map_frame",         map_frame);
  
  ROS_INFO("Perception: ");
  ROS_INFO("  RGB image topic:   %s.", rgb_image_topic.c_str());
  ROS_INFO("  Depth image topic: %s.", depth_image_topic.c_str());
  ROS_INFO("  Camera info topic: %s.", camera_info_topic.c_str());
  ROS_INFO("  Camera frame:      %s.", camera_frame.c_str());
  ROS_INFO("  Map frame:         %s.", map_frame.c_str());
  
  // Get local parameters
  nh_private.param("mipmap_level", mipmap_level, 0);
  nh_private.param("max_objects_per_frame", max_objects_per_frame, 1);
  nh_private.param("max_nof_keypoints", max_nof_keypoints, 64);
  ROS_INFO("  Mipmap level: %i.", mipmap_level);
  ROS_INFO("  Max objects per frame: %i.", max_objects_per_frame);
  ROS_INFO("  Max number of keypoints: %i.", max_nof_keypoints);
  
  // Initialize reusable service clients
  ros::service::waitForService("thesis_database/get_all", -1);
  db_get_all_client = nh.serviceClient<thesis::DatabaseGetAll>("thesis_database/get_all");
  ros::service::waitForService("thesis_database/set_by_type", -1);
  db_set_by_type_client = nh.serviceClient<thesis::DatabaseSetByID>("thesis_database/set_by_type");
  
  // Create image database
  reset(mipmap_level);

  // Subscribe to relevant OpenNI topics
  Subscriber<Image> rgb_subscriber(nh, rgb_image_topic, 1);
  Subscriber<Image> depth_subscriber(nh, depth_image_topic, 1);
  Subscriber<CameraInfo> cam_info_subscriber(nh, camera_info_topic, 1);
  // Use one time-sychronized callback for all OpenNI subscriptions
  typedef sync_policies::ApproximateTime<Image, Image, CameraInfo> SyncPolicy;
  Synchronizer<SyncPolicy> synchronizer(SyncPolicy(10), rgb_subscriber, depth_subscriber, cam_info_subscriber);
  synchronizer.registerCallback(boost::bind(&callback_openni, _1, _2, _3));
  // Subscribe to updates about the database
  ros::Subscriber database_update_subscriber = nh.subscribe("thesis_database/updates", 1, callback_database_update);
  // Publish recognized objects
  object_publisher = nh_private.advertise<thesis::ObjectStamped>("objects", 1000);
  // Spin
  ros::spin();
  // Free memory
  cv::destroyWindow(CAMERA_DEBUG_IMAGE_WINDOW);
  cv::destroyWindow(MIPMAP_DEBUG_IMAGE_WINDOW);
  // Exit with success
  return 0;
}
