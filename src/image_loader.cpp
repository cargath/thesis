/**
 * @author Carsten Könemann
 */

#include <thesis/image_loader.h>

#include <ros/ros.h>

ImageLoader::ImageLoader()
{
  // Default constructor
}

ImageLoader::~ImageLoader()
{
  // Default destructor
}

bool ImageLoader::load(const std::string& path, cv::Mat& out_image)
{
  out_image = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
  if(!out_image.empty())
  {
    ROS_INFO("Loading '%s'.", path.c_str());
    return true;
  }
  else
  {
    ROS_ERROR("Unable to load image '%s'.", path.c_str());
    return false;
  }
}

bool ImageLoader::load_image(boost::filesystem::path path, cv::Mat& out_image)
{
  if(boost::filesystem::exists(path) && !boost::filesystem::is_directory(path))
  {
    return load((std::string) path.string().c_str(), out_image);
  }
  else
  {
    ROS_ERROR("Unable to load image '%s'.", path.string().c_str());
    return false;
  }
}

bool ImageLoader::load_directory(boost::filesystem::path path,
                                 std::vector<cv::Mat>& out_images,
                                 std::vector<std::string>* filenames)
{
  if(boost::filesystem::exists(path) && boost::filesystem::is_directory(path))
  {
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
          cv::Mat image;
          if(load(itr->path().string(), image))
          {
            out_images.push_back(image);
            if(filenames != NULL)
            {
              (*filenames).push_back(file_name);
            }
          }
        }
      }
    }
    return true;
  }
  else
  {
    ROS_ERROR("Directory '%s' not found.", path.string().c_str());
    return false;
  }
}
