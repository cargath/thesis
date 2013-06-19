/**
 * @author Carsten Könemann
 */

// This is a ROS project
#include <ros/ros.h>

// Since we use OpenCV to load image files,
// we need to convert them to ROS messages
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// We try to get the image size of an OpenNI camera (if available)
#include <sensor_msgs/CameraInfo.h>

// Local headers
#include <thesis/image_loader.h>

// This node provides these services
#include <thesis/DatabaseAddDir.h>
#include <thesis/DatabaseAddFile.h>
#include <thesis/DatabaseAddImg.h>
#include <thesis/DatabaseGetAll.h>
#include <thesis/DatabaseGetByID.h>
#include <thesis/DatabaseList.h>
#include <thesis/DatabaseSetByID.h>

// Constants
static const double MAX_OPENNI_TOPIC_WAIT_TIME = 5.0;

ImageLoader image_loader;

// We try to get the image size of a connected OpenNI camera (if available)
bool openni_once = false;
cv::Size openni_image_size;

// Image size limits
cv::Size min_image_size,
         max_image_size;

// Store all sample objects mapped to their IDs for quick access
typedef std::map<std::string, thesis::Sample> SampleMap;
SampleMap samples;

bool add_image(cv::Mat& image, std::string name)
{
  // Check if image is large enough
  if(image.cols < min_image_size.width || image.rows < min_image_size.height)
  {
    ROS_WARN("Image '%s' is smaller than %i x %i.",
             name.c_str(), min_image_size.width, min_image_size.height);
    ROS_WARN("Don't add it to database.");
    return false;
  }
  // Determine if we need max_image_size in portrait or landscape orientation
  // in order to match the sample image orientation
  if(max_image_size.width < max_image_size.height)
  {
    max_image_size = (image.cols <  image.rows) ? max_image_size : cv::Size(max_image_size.height, max_image_size.width);
  }
  else
  {
    max_image_size = (image.cols >= image.rows) ? max_image_size : cv::Size(max_image_size.height, max_image_size.width);
  }
  // Resize image (if too large)
  cv::Mat image_resized;
  if(image.cols > max_image_size.width || image.rows > max_image_size.height)
  {
    // ...determine biggest possible new size with the same aspect ratio
    cv::Size re_size;
    float aspect_ratio_width  = (float) max_image_size.width  / (float) image.cols,
          aspect_ratio_height = (float) max_image_size.height / (float) image.rows;
    if(aspect_ratio_width < aspect_ratio_height)
    {
      re_size = cv::Size(image.cols * aspect_ratio_width,
                         image.rows * aspect_ratio_width);
    }
    else
    {
      re_size = cv::Size(image.cols * aspect_ratio_height,
                         image.rows * aspect_ratio_height);
    }
    // ...and resize it accordingly
    ROS_WARN("Image '%s' is bigger than %i x %i.", name.c_str(), max_image_size.width, max_image_size.height);
    ROS_WARN("Resizing it to %i x %i.", re_size.width, re_size.height);
    cv::resize(image, image_resized, re_size);
  }
  else
  {
    image_resized = image;
  }
  // Create empty sample
  thesis::Sample sample;
  sample.id     = name;
  sample.width  = 0;
  sample.height = 0;
  // Convert OpenCV image to ROS image message
  cv_bridge::CvImage cv_image;
  cv_image.encoding = sensor_msgs::image_encodings::MONO8;
  cv_image.image    = image_resized;
  cv_image.toImageMsg(sample.image);
  // Add sample to database
  samples[sample.id] = sample;
  // Success
  return true;
}

void callback_openni_once(const sensor_msgs::CameraInfo::ConstPtr& input)
{
  openni_image_size.width  = input->width;
  openni_image_size.height = input->height;
  openni_once = true;
}

bool add_directory(thesis::DatabaseAddDir::Request& request,
                   thesis::DatabaseAddDir::Response& result)
{
  // TODO
  return true;
}

bool add_file(thesis::DatabaseAddFile::Request& request,
              thesis::DatabaseAddFile::Response& result)
{
  // TODO
  return true;
}

bool add_image(thesis::DatabaseAddImg::Request& request,
               thesis::DatabaseAddImg::Response& result)
{
  // TODO
  return true;
}

bool get_list(thesis::DatabaseList::Request& request,
              thesis::DatabaseList::Response& result)
{
  for(SampleMap::iterator it = samples.begin(); it != samples.end(); it++)
  {
    result.list.push_back(it->first);
  }
  return true;
}

bool get_all(thesis::DatabaseGetAll::Request& request,
             thesis::DatabaseGetAll::Response& result)
{
  for(SampleMap::iterator it = samples.begin(); it != samples.end(); it++)
  {
    result.samples.push_back(it->second);
  }
  return true;
}

bool get_by_type(thesis::DatabaseGetByID::Request& request,
                 thesis::DatabaseGetByID::Response& result)
{
  result.sample = samples[request.id];
  return true;
}

bool set_by_type(thesis::DatabaseSetByID::Request& request,
                 thesis::DatabaseSetByID::Response& result)
{
  thesis::Sample temp = samples[request.sample.id];
  if(temp.accuracy < INT_MAX-1)
  {
    samples[request.sample.id].accuracy++;
  }
  samples[request.sample.id].width  = (temp.width  * temp.accuracy + request.sample.width)  / (temp.accuracy + 1);
  samples[request.sample.id].height = (temp.height * temp.accuracy + request.sample.height) / (temp.accuracy + 1);
  return true;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "thesis_database");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  // Get path to image directory
  std::string image_path;
  nh_private.param("image_path", image_path, std::string("img"));
  // Try to get OpenNI camera image size
  std::string camera_info_topic;
  nh.param("camera_info_topic", camera_info_topic, std::string("camera/depth_registered/camera_info"));
  ros::Subscriber camera_info_subscriber = nh.subscribe(camera_info_topic, 1, callback_openni_once);
  ros::Time wait_time = ros::Time::now();
  while(!openni_once)
  {
    ros::spinOnce();
    if(ros::Time::now().toSec() - wait_time.toSec() > MAX_OPENNI_TOPIC_WAIT_TIME)
    {
      break;
    }
  }
  camera_info_subscriber.shutdown();
  // Set image size limits
  nh_private.param("min_image_width",  min_image_size.width,  1);
  nh_private.param("min_image_height", min_image_size.height, 1);
  if(openni_once)
  {
    ROS_ASSERT_MSG(openni_image_size.width > 0 && openni_image_size.height > 0,
                   "Got bad image size from OpenNI camera.");
    max_image_size.width  = openni_image_size.width;
    max_image_size.height = openni_image_size.height;
    ROS_INFO("Got the following image resolution from OpenNI camera: %ix%i",
             openni_image_size.width, openni_image_size.height);
  }
  else
  {
    nh_private.param("max_image_width",  max_image_size.width,  1280);
    nh_private.param("max_image_height", max_image_size.height, 1024);
    ROS_INFO("Unable to get image resolution from OpenNI camera.");
    ROS_INFO("Using default or given max resolution (%ix%i) instead.",
             max_image_size.width, max_image_size.height);
  }
  // Create sample database
  std::vector<cv::Mat> images;
  std::vector<std::string> filenames;
  image_loader.load_directory(image_path, images, &filenames);
  ROS_ASSERT_MSG(images.size() == filenames.size(),
                 "Number of images should be the same as number of filenames.");
  for(size_t i = 0; i < images.size(); i++)
  {
    add_image(images[i], filenames[i]);
  }
  // Advertise services
  ros::ServiceServer srv_add_directory = nh_private.advertiseService("add_directory", add_directory);
  ros::ServiceServer srv_add_file      = nh_private.advertiseService("add_file",      add_file);
  ros::ServiceServer srv_add_image     = nh_private.advertiseService("add_image",     add_image);
  ros::ServiceServer srv_get_list      = nh_private.advertiseService("get_list",      get_list);
  ros::ServiceServer srv_get_all       = nh_private.advertiseService("get_all",       get_all);
  ros::ServiceServer srv_get_by_type   = nh_private.advertiseService("get_by_type",   get_by_type);
  ros::ServiceServer srv_set_by_type   = nh_private.advertiseService("set_by_type",   set_by_type);
  // Spin
  ros::spin();
  // Exit
  return 0;
}
