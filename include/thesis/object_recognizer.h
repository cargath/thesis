/**
 * @author Carsten Könemann
 */
#ifndef __OBJECT_RECOGNIZER__
#define __OBJECT_RECOGNIZER__

#include <thesis/Sample.h>

#include <thesis/color.h>
#include <thesis/dbscan.h>

#include <opencv2/nonfree/features2d.hpp>

class ObjectRecognizer
{
  public:
    struct ProcessedSample
    {
      std::string               id;
      int                       width,
                                height;
      std::vector<cv::KeyPoint> keypoints;
      cv::Mat                   descriptors;
    };
  
    struct Finding
    {
      std::string              id;
      std::vector<cv::Point2f> image_points;
    };
  
    // Default constructor
    ObjectRecognizer();
    // Default destructor
    ~ObjectRecognizer();

    // Process and add a sample image to the database
    ProcessedSample processSample(const cv::Mat& image, const std::string& id="");
    
    // Detect given object in given image
    void detectObject(const std::vector<cv::KeyPoint>& keypoints,
                      const cv::Mat& descriptors,
                      const ProcessedSample& object,
                      Finding& finding,
                      cv::Mat* debug_image=NULL);
    void detectObject(const cv::Mat& image,
                      const ProcessedSample& object,
                      Finding& finding,
                      cv::Mat* debug_image=NULL);
    // Detect objects from given database in given image
    void detectObjects(const cv::Mat& image,
                       const std::vector<ProcessedSample>& objects,
                       std::vector<Finding>& findings,
                       cv::Mat* debug_image=NULL);
    // Cluster keypoints for smarter object recognition (experimental)
    void detectObjectsClustered(const cv::Mat& image,
                                const std::vector<ProcessedSample>& objects,
                                std::vector<Finding>& findings,
                                cv::Mat* debug_image=NULL);
    
    // Draw a single match
    void drawMatch(cv::Mat& img,
                   const cv::DMatch& match,
                   const std::vector<cv::KeyPoint>& a,
                   const std::vector<cv::KeyPoint>& b,
                   const cv::Scalar& color=WHITE,
                   int thickness=1,
                   int line_type=8);
    // Draw multiple matches
    void drawMatches(cv::Mat& img,
                     const std::vector<cv::DMatch>& matches,
                     const std::vector<cv::KeyPoint>& a,
                     const std::vector<cv::KeyPoint>& b,
                     const cv::Scalar& color=WHITE,
                     int thickness=1,
                     int line_type=8);

  protected:
    // Associates a cluster of matches with the corresponding keypoint clusters
    struct DMatchCluster
    {
      int matchIdx, queryIdx, trainIdx;
  
      DMatchCluster(int mi, int qi, int ti)
      {
        matchIdx = mi;
        queryIdx = qi;
        trainIdx = ti;
      }
    };
    
    // Reusable OpenCV stuff for working with images
    cv::SurfFeatureDetector feature_detector;
    cv::SurfDescriptorExtractor descriptor_extractor;
    cv::FlannBasedMatcher descriptor_matcher;
    DBScanner dbscanner;
};

#endif //__OBJECT_RECOGNIZER__
