/**
 *
 * @author Carsten Könemann
 */

#include <thesis/object_database.h>

#include <ros/ros.h>

ObjectDatabase::ObjectDatabase()
{
  // Default constructor
}

ObjectDatabase::~ObjectDatabase()
{
  // Default destructor
}

ObjectDatabase::ObjectDatabase(boost::filesystem::path path)
{
  load_directory(path);
}

bool ObjectDatabase::load_image(std::string path)
{
  cv::Mat image = cv::imread(path);
  if(!image.empty())
  {
    ROS_INFO("Loading '%s'.", path.c_str());
    // Detect keypoints
    std::vector<cv::KeyPoint> keypoints;
    feature_detector.detect(image, keypoints);
    // Compute descriptors
    cv::Mat descriptors;
    descriptor_extractor.compute(image, keypoints, descriptors);
    // Add object to database
    objects.push_back(SampleObject(boost::filesystem::basename(path), image, keypoints, descriptors));
    return true;
  }
  else
  {
    ROS_ERROR("Unable to load image '%s'.", path.c_str());
    return false;
  }
}

bool ObjectDatabase::load_image(boost::filesystem::path path)
{
  if(boost::filesystem::exists(path) && !boost::filesystem::is_directory(path))
  {
    return load_image((std::string) path.string().c_str());
  }
  else
  {
    ROS_ERROR("Unable to load image '%s'.", path.string().c_str());
    return false;
  }
}

bool ObjectDatabase::load_directory(boost::filesystem::path path)
{
  if(boost::filesystem::exists(path) && boost::filesystem::is_directory(path))
  {
    bool return_value = true;
    // Check all elements contained in given directory
    boost::filesystem::directory_iterator end_itr;
    for(boost::filesystem::directory_iterator itr(path); itr != end_itr; itr++)
    {
      // Only load try loading files, not directories
      if(!boost::filesystem::is_directory(itr->status()))
      {
        std::string file_name = boost::filesystem::basename(itr->path().filename());
        if(file_name.length() > 0)
        {
          load_image(itr->path().string());
        }
      }
    }
    return return_value;
  }
  else
  {
    ROS_ERROR("Directory '%s' not found.", path.string().c_str());
    return false;
  }
}
