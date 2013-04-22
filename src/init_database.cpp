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
#include <thesis/DatabaseList.h>
#include <thesis/DatabaseGetByID.h>

ImageLoader image_loader;

// Store all sample objects mapped to their IDs for quick access
typedef std::map<std::string, thesis::Sample> SampleMap;
SampleMap samples;

bool list(thesis::DatabaseList::Request& request,
          thesis::DatabaseList::Response& result)
{
  for(SampleMap::iterator it = samples.begin(); it != samples.end(); it++)
  {
    result.list.push_back(it->first);
  }
  return true;
}

bool get(thesis::DatabaseGetByID::Request& request,
         thesis::DatabaseGetByID::Response& result)
{
  result.sample = samples[request.id];
  return true;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "thesis_database");
  ros::NodeHandle nh;
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
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
    cv_ptr->image    = images[i];
    cv_ptr->toImageMsg(sample.image);
  }
  // Advertise services
  ros::ServiceServer srv_list = nh.advertiseService("thesis/database/list", list);
  ros::ServiceServer srv_get  = nh.advertiseService("thesis/database/getByID", get);
  // Spin
  ros::spin();
  // Exit
  return 0;
}
