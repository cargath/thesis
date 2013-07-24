/**
 * @author Carsten Könemann
 */
#ifndef __OBJECT_RECOGNIZER__
#define __OBJECT_RECOGNIZER__

#include <thesis/color.h>

#ifdef  USE_SIFT_GPU
  #include <thesis/sift_gpu_wrapper.h>
#endif
#ifndef USE_SIFT_GPU
  #include <opencv2/nonfree/features2d.hpp>
#endif

class ObjectRecognizer
{
  public:
    // Contains all the information we need about an image
    struct ImageInfo
    {
      int                       width,
                                height;
      std::vector<cv::KeyPoint> keypoints;
      #ifdef  USE_SIFT_GPU
        std::vector<float>      descriptors;
      #endif
      #ifndef USE_SIFT_GPU
        cv::Mat                 descriptors;
      #endif
    };
  
    // Default constructor
    ObjectRecognizer();
    // Default destructor
    ~ObjectRecognizer();

    /**
     */
    void setMaxKeypoints(const int maxKeypoints);

    /**
     */
    void filterImageInfo(const ImageInfo& input,
                         const std::vector<cv::Point2f>& mask,
                         ImageInfo* inside_mask,
                         ImageInfo* outside_mask);

    /**
     */
    void getImageInfo(const cv::Mat& image, ImageInfo& image_info);
    
    /**
     */
    bool recognize(ImageInfo& sample_info,
                   ImageInfo& cam_img_info,
                   std::vector<cv::Point2f>& object_points,
                   cv::FlannBasedMatcher* matcher=NULL);

  protected:
    #ifndef USE_SIFT_GPU
      // Reusable OpenCV stuff for working with images
      cv::SiftFeatureDetector feature_detector;
      cv::SiftDescriptorExtractor descriptor_extractor;
      cv::FlannBasedMatcher flann_matcher;
    #endif
};

#endif //__OBJECT_RECOGNIZER__
