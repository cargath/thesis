/**
 * @author Carsten Könemann
 */
#ifndef __OBJECT_RECOGNIZER__
#define __OBJECT_RECOGNIZER__

#include <thesis/color.h>

#include <opencv2/nonfree/features2d.hpp>

class ObjectRecognizer
{
  public:
    // Contains all the information we need about an image
    struct ImageInfo
    {
      int                       width,
                                height;
      std::vector<cv::KeyPoint> keypoints;
      cv::Mat                   descriptors;
      cv::FlannBasedMatcher     matcher;
    };
  
    // Default constructor
    ObjectRecognizer();
    // Default destructor
    ~ObjectRecognizer();

    // Process a sample image (compute keypoints and descriptors)
    void getImageInfo(const cv::Mat& image, ImageInfo& image_info);
    
    //
    void recognize(ImageInfo& sample_info,
                   const cv::Mat& camera_image,
                   std::vector<cv::Point2f>& object_points,
                   cv::Mat* debug_image=NULL);
    
    //
    void recognize(ImageInfo& sample_info,
                   ImageInfo& cam_img_info,
                   std::vector<cv::Point2f>& object_points,
                   cv::Mat* debug_image=NULL);

  protected:
    // Reusable OpenCV stuff for working with images
    cv::SiftFeatureDetector feature_detector;
    cv::SiftDescriptorExtractor descriptor_extractor;
};

#endif //__OBJECT_RECOGNIZER__
