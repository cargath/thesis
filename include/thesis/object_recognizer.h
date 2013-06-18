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

    // Use the object recognizers feature detector to compute keypoints.
    // Call this method if you want to make sure to get compatible keypoints.
    void getKeypoints(const cv::Mat& image,
                      std::vector<cv::KeyPoint>& keypoints);
    
    // Use the object recognizers descriptor extractor to compute descriptors.
    // Call this method if you want to make sure to get compatible descriptors.
    void getDescriptors(const cv::Mat& image,
                        std::vector<cv::KeyPoint>& keypoints,
                        cv::Mat& descriptors);

    // Process a sample image (compute keypoints and descriptors)
    void getImageInfo(const cv::Mat& image,
                      ImageInfo& image_info,
                      std::vector<cv::KeyPoint>* keypoints=NULL,
                      cv::Mat* descriptors=NULL);
    
    // Process a rectangular area of an image given by four corner points
    void getPartialImageInfo(const cv::Mat& image,
                             const std::vector<cv::Point2f>& corners,
                             ImageInfo& image_info,
                             std::vector<cv::KeyPoint>* keypoints=NULL,
                             cv::Mat* descriptors=NULL);
    
    //
    void copyImageInfo(const ImageInfo& from, ImageInfo& to);
    
    //
    bool recognize(ImageInfo& sample_info,
                   ImageInfo& cam_img_info,
                   std::vector<cv::Point2f>& object_points);

  protected:
    // Reusable OpenCV stuff for working with images
    cv::SiftFeatureDetector feature_detector;
    cv::SiftDescriptorExtractor descriptor_extractor;
};

#endif //__OBJECT_RECOGNIZER__
