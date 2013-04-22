/**
 *
 * @author Carsten Könemann
 */
#ifndef __OBJECT_DATABASE__
#define __OBJECT_DATABASE__

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <boost/filesystem.hpp>

class ObjectDatabase
{
  public:
    struct SampleObject
    {
      std::string name;
      cv::Mat img, desc;
      std::vector<cv::KeyPoint> keypoints;
      
      SampleObject(std::string n, cv::Mat i, std::vector<cv::KeyPoint> kp, cv::Mat d)
      {
        name = n;
        img = i;
        keypoints = kp;
        desc = d;
      }
    };

    // All objects belonging to this database.
    // I know this should be protected and accessed via getter,
    // but it should be even faster than returning a reference this way.
    // (Depending on the compiler?)
    std::vector<SampleObject> objects;

    // Default constructor
    ObjectDatabase();
    // Default destructor
    ~ObjectDatabase();

    // Constructor reading some sample images on startup
    ObjectDatabase(boost::filesystem::path path);
    
    // Read a single sample image
    bool load_image(std::string path);
    bool load_image(boost::filesystem::path path);
    // Read all sample images from a directory
    bool load_directory(boost::filesystem::path path);
    
  protected:
    // Reusable OpenCV stuff for working with images
    cv::SurfFeatureDetector feature_detector;
    cv::SurfDescriptorExtractor descriptor_extractor;
};

#endif //__OBJECT_DATABASE__
