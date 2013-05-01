/**
 * @author Carsten Könemann
 */

// This is a ROS project
#include <ros/ros.h>

// 
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Local headers
#include <thesis/image_loader.h>

// This node provides these services
#include <thesis/DatabaseGetAll.h>
#include <thesis/DatabaseGetByID.h>
#include <thesis/DatabaseList.h>

ImageLoader image_loader;

// Store all sample objects mapped to their IDs for quick access
typedef std::map<std::string, thesis::Sample> SampleMap;
SampleMap samples;

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

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "thesis_database");
  ros::NodeHandle nh_private("~");
  // Create sample database
  std::vector<cv::Mat> images;
  std::vector<std::string> filenames;
  image_loader.load_directory("img", images, &filenames);
  ROS_ASSERT_MSG(images.size() == filenames.size(), "#images should be the same as #filenames.");
  for(size_t i = 0; i < images.size(); i++)
  {
    thesis::Sample sample;
    sample.id = filenames[i];
    // Convert OpenCV image to ROS image message
    cv_bridge::CvImage cv_image;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image.image    = images[i];
    cv_image.toImageMsg(sample.image);
    // Add to database
    samples[sample.id] = sample;
  }
  // Advertise services
  ros::ServiceServer srv_list    = nh_private.advertiseService("list", get_list);
  ros::ServiceServer srv_all     = nh_private.advertiseService("all", get_all);
  ros::ServiceServer srv_by_type = nh_private.advertiseService("by_type", get_by_type);
  // Spin
  ros::spin();
  // Exit
  return 0;
}
