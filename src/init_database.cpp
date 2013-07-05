/**
 * @author Carsten Könemann
 */

// This is a ROS project
#include <ros/ros.h>

// Local headers
#include <thesis/image_loader.h>

// Since we use OpenCV to load image files,
// we need to convert them to ROS messages
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// We try to get the image size of an OpenNI camera (if available)
#include <sensor_msgs/CameraInfo.h>

// Empty message to inform subscribing nodes about changes
#include <std_msgs/Empty.h>

// This node provides these services
#include <thesis/DatabaseAddFile.h>
#include <thesis/DatabaseAddImg.h>
#include <thesis/DatabaseAddURL.h>
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

// We are going to inform subscribing nodes about changes
ros::Publisher update_publisher;

// Check if an entry for this ID exists in the database
bool exists(std::string id)
{
  try
  {
    samples.at(id);
  }
  catch(const std::out_of_range& oor)
  {
    return false;
  }
  return true;
}

bool add_image(cv::Mat& image, std::string name)
{
  // Check if image name is valid
  if(!(name.length() > 0))
  {
    ROS_WARN("Error while trying to add an image to the database:");
    ROS_WARN("  Image name empty.");
    ROS_WARN("  Don't add it to the database.");
    return false;
  }
  // Check if an image of this name already exists in the database
  if(exists(name))
  {
    ROS_WARN("Error while trying to add an image to the database:");
    ROS_WARN("  An image of the name '%s' already exists in the database.", name.c_str());
    ROS_WARN("  Don't add it to the database again.");
    return false;
  }
  // Check if image is large enough
  if(image.cols < min_image_size.width || image.rows < min_image_size.height)
  {
    ROS_WARN("Error while trying to add an image to the database:");
    ROS_WARN("  Image '%s' is smaller than %i x %i.",
             name.c_str(), min_image_size.width, min_image_size.height);
    ROS_WARN("  Don't add it to database.");
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
    ROS_INFO("While trying to add an image to the database:");
    ROS_INFO("  Image '%s' is bigger than %i x %i.", name.c_str(), max_image_size.width, max_image_size.height);
    ROS_INFO("  Resizing it to %i x %i.", re_size.width, re_size.height);
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

bool add_image(sensor_msgs::Image& image, std::string name)
{
  // Convert ROS image message to OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }
  // Add OpenCV image
  return add_image(cv_ptr->image, name);
}

void callback_openni_once(const sensor_msgs::CameraInfo::ConstPtr& input)
{
  openni_image_size.width  = input->width;
  openni_image_size.height = input->height;
  openni_once = true;
}

bool callback_add_urls(thesis::DatabaseAddURL::Request& request,
                       thesis::DatabaseAddURL::Response& result)
{
  for(size_t i = 0; i < request.urls.size(); i++)
  {
    std::vector<cv::Mat> images;
    std::vector<std::string> filenames;
    if(image_loader.load_url(request.urls[i], images, filenames))
    {
      for(size_t j = 0; j < images.size(); j++)
      {
        add_image(images[j], filenames[j]);
      }
    }
  }
  return true;
}

bool callback_add_files(thesis::DatabaseAddFile::Request& request,
                        thesis::DatabaseAddFile::Response& result)
{
  for(size_t i = 0; i < request.urls.size(); i++)
  {
    cv::Mat image;
    std::string filename;
    if(image_loader.load_file(request.urls[i], image, filename))
    {
      if(request.names.size() >= i && request.names[i].length() > 0)
      {
        add_image(image, request.names[i]);
      }
      else
      {
        ROS_INFO("Add file service:");
        ROS_INFO("  Image name is empty. Using filename ('%s') instead.", filename.c_str());
        add_image(image, filename);
      }
    }
  }
  return true;
}

bool callback_add_images(thesis::DatabaseAddImg::Request& request,
                         thesis::DatabaseAddImg::Response& result)
{
  for(size_t i = 0; i < request.images.size(); i++)
  {
    add_image(request.images[i], request.names[i]);
  }
  return true;
}

bool callback_get_list(thesis::DatabaseList::Request& request,
                       thesis::DatabaseList::Response& result)
{
  for(SampleMap::iterator it = samples.begin(); it != samples.end(); it++)
  {
    result.list.push_back(it->first);
  }
  return true;
}

bool callback_get_all(thesis::DatabaseGetAll::Request& request,
                      thesis::DatabaseGetAll::Response& result)
{
  for(SampleMap::iterator it = samples.begin(); it != samples.end(); it++)
  {
    result.samples.push_back(it->second);
  }
  return true;
}

bool callback_get_by_type(thesis::DatabaseGetByID::Request& request,
                          thesis::DatabaseGetByID::Response& result)
{
  result.sample = samples[request.id];
  return true;
}

bool callback_set_by_type(thesis::DatabaseSetByID::Request& request,
                          thesis::DatabaseSetByID::Response& result)
{
  if(exists(request.sample.id))
  {
    thesis::Sample temp = samples[request.sample.id];
    if(temp.accuracy < INT_MAX - 2)
    {
      samples[request.sample.id].accuracy++;
    }
    samples[request.sample.id].width  = (temp.width  * temp.accuracy + request.sample.width)  / (temp.accuracy + 1);
    samples[request.sample.id].height = (temp.height * temp.accuracy + request.sample.height) / (temp.accuracy + 1);
    return true;
  }
  else
  {
    ROS_WARN("Update entry service:");
    ROS_WARN("  Could not find an entry for '%s'.", request.sample.id.c_str());
    return false;
  }
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
    ROS_INFO("Initializing image resolution limits:");
    ROS_INFO("  Got the following image resolution from OpenNI camera: %ix%i",
             openni_image_size.width, openni_image_size.height);
  }
  else
  {
    nh_private.param("max_image_width",  max_image_size.width,  1280);
    nh_private.param("max_image_height", max_image_size.height, 1024);
    ROS_INFO("Initializing image resolution limits:");
    ROS_INFO("  Unable to get image resolution from OpenNI camera.");
    ROS_INFO("  Using default or given max resolution (%ix%i) instead.",
             max_image_size.width, max_image_size.height);
  }
  // Create sample database
  std::vector<cv::Mat> images;
  std::vector<std::string> filenames;
  image_loader.load_directory(image_path, images, filenames);
  for(size_t i = 0; i < images.size(); i++)
  {
    add_image(images[i], filenames[i]);
  }
  // Advertise services
  ros::ServiceServer srv_add_urls    = nh_private.advertiseService("add_urls",    callback_add_urls);
  ros::ServiceServer srv_add_files   = nh_private.advertiseService("add_files",   callback_add_files);
  ros::ServiceServer srv_add_images  = nh_private.advertiseService("add_images",  callback_add_images);
  ros::ServiceServer srv_get_list    = nh_private.advertiseService("get_list",    callback_get_list);
  ros::ServiceServer srv_get_all     = nh_private.advertiseService("get_all",     callback_get_all);
  ros::ServiceServer srv_get_by_type = nh_private.advertiseService("get_by_type", callback_get_by_type);
  ros::ServiceServer srv_set_by_type = nh_private.advertiseService("set_by_type", callback_set_by_type);
  // We are going to inform subscribing nodes about changes
  update_publisher = nh_private.advertise<std_msgs::Empty>("updates", 10);
  // Spin
  ros::spin();
  // Exit
  return 0;
}
