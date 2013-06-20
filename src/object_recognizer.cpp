/**
 * @author Carsten Könemann
 */

#include <thesis/object_recognizer.h>

#include <thesis/clock.h>
#include <thesis/graham_scanner.h>
#include <thesis/math2d.h>

#include <opencv2/flann/flann.hpp>

ObjectRecognizer::ObjectRecognizer()
{
  // Default constructor
}

ObjectRecognizer::~ObjectRecognizer()
{
  // Default destructor
}

inline bool x0r(bool a, bool b)
{
  return (a && !b) || (b && !a);
}

void ObjectRecognizer::getKeypoints(const cv::Mat& image,
                                    std::vector<cv::KeyPoint>& keypoints)
{
  feature_detector.detect(image, keypoints);
}

void ObjectRecognizer::getDescriptors(const cv::Mat& image,
                                      std::vector<cv::KeyPoint>& keypoints,
                                      cv::Mat& descriptors)
{
  if(!keypoints.empty())
  {
    descriptor_extractor.compute(image, keypoints, descriptors);
  }
}

void ObjectRecognizer::getImageInfo(const cv::Mat& image,
                                    ImageInfo& image_info,
                                    std::vector<cv::KeyPoint>* keypoints,
                                    cv::Mat* descriptors)
{
  image_info.width  = image.cols;
  image_info.height = image.rows;
  // Detect keypoints
  if(keypoints)
  {
    image_info.keypoints = *keypoints;
  }
  else
  {
    feature_detector.detect(image, image_info.keypoints);
  }
  // Compute descriptors
  if(descriptors)
  {
    image_info.descriptors = *descriptors;
  }
  else
  {
    if(!image_info.keypoints.empty())
    {
      descriptor_extractor.compute(image, image_info.keypoints, image_info.descriptors);
    }
  }
  // Train matcher
  if(!image_info.descriptors.empty())
  {
    std::vector<cv::Mat> descriptor_vector;
    descriptor_vector.push_back(image_info.descriptors);
    image_info.matcher.clear();
    image_info.matcher.add(descriptor_vector);
    image_info.matcher.train();
  }
}

void ObjectRecognizer::getPartialImageInfo(const cv::Mat& image,
                                           const std::vector<cv::Point2f>& corners,
                                           ImageInfo& image_info,
                                           std::vector<cv::KeyPoint>* keypoints,
                                           cv::Mat* descriptors,
                                           std::vector<cv::KeyPoint>* keypoints_cut,
                                           cv::Mat* descriptors_cut)
{
  // Detect all keypoints (if none are given)
  std::vector<cv::KeyPoint> keypoints_all;
  if(keypoints)
  {
    keypoints_all = *keypoints;
  }
  else
  {
    feature_detector.detect(image, keypoints_all);
  }
  // Only use keypoints located inside the rectangle defined by the given corners
  std::vector<cv::KeyPoint> keypoints_filtered;
  cv::Mat descriptors_filtered;
  for(size_t i = 0; i < keypoints_all.size(); i++)
  {
    if(insideConvexPolygon(corners, keypoints_all[i].pt))
    {
      keypoints_filtered.push_back(keypoints_all[i]);
      if(descriptors)
      {
        descriptors_filtered.push_back(descriptors->row(i));
      }
    }
    else
    {
      if(keypoints_cut)
      {
        keypoints_cut->push_back(keypoints_all[i]);
      }
      if(descriptors_cut)
      {
        descriptors_cut->push_back(descriptors->row(i));
      }
    }
  }
  // Return image info for all applicable keypoints
  if(descriptors)
  {
    getImageInfo(image, image_info, &keypoints_filtered, &descriptors_filtered);
  }
  else
  {
    getImageInfo(image, image_info, &keypoints_filtered);
  }
}

void ObjectRecognizer::copyImageInfo(const ImageInfo& from, ImageInfo& to)
{
  to.width       = from.width;
  to.height      = from.height;
  to.keypoints   = from.keypoints;
  to.descriptors = from.descriptors;
  to.matcher     = from.matcher;
}

bool ObjectRecognizer::recognize(ImageInfo& sample_info,
                                 ImageInfo& cam_img_info,
                                 std::vector<cv::Point2f>& object_points)
{
  std::vector<cv::Mat> train_descriptors = cam_img_info.matcher.getTrainDescriptors();
  // Otherwise an OpenCV assertion would fail for images without keypoints
  if(cam_img_info.matcher.empty()
  || train_descriptors.front().rows < 2
  || sample_info.descriptors.rows   < 2)
  {
    return false;
  }
  // Find the k = 2 nearest neighbours
  std::vector<std::vector<cv::DMatch> > matches;
  cam_img_info.matcher.knnMatch(sample_info.descriptors, matches, 2);
  // Filter matches:
  // By ratio of nearest and second nearest neighbour distance
  std::vector<cv::DMatch> matches_filtered;
  for(size_t i = 0; i < matches.size(); i++)
  {
    float ratio = matches[i][0].distance / matches[i][1].distance;
    if(ratio < 0.8f)
    {
      matches_filtered.push_back(matches[i][0]);
    }
  }
  // Locate objects
  if(matches_filtered.size() >= 4)
  {
    std::vector<cv::Point2f> object_in_scene,
                             scene;
    for(size_t i = 0; i < matches_filtered.size(); i++)
    {
      object_in_scene.push_back(sample_info.keypoints[matches_filtered[i].queryIdx].pt);
      scene.push_back(cam_img_info.keypoints[matches_filtered[i].trainIdx].pt);
    }
    // Compute homography
    cv::Mat homography = cv::findHomography(object_in_scene, scene, CV_RANSAC);
    // Compute perspective transform
    std::vector<cv::Point2f> object_corners = std::vector<cv::Point2f>(4),
                             scene_corners  = std::vector<cv::Point2f>(4);
    object_corners[0] = cvPoint(                0,                  0);
    object_corners[1] = cvPoint(sample_info.width,                  0);
    object_corners[2] = cvPoint(sample_info.width, sample_info.height);
    object_corners[3] = cvPoint(                0, sample_info.height);
    cv::perspectiveTransform(object_corners, scene_corners, homography);
    // Add object points to output before filtering false positives
    // (we still might want to visualize them for debugging purposes)
    object_points = scene_corners;
    // Filter false positives:
    // Check if object corners in scene are twisted
    #define p0 scene_corners[0]
    #define p1 scene_corners[1]
    #define p2 scene_corners[2]
    #define p3 scene_corners[3]
    cv::Point2f intersection;
    if(intersectLineSegments(p0, p1, p2, p3, intersection)
    || intersectLineSegments(p0, p3, p1, p2, intersection))
    {
      return false;
    }
    // Filter false positives:
    // Check if set of scene points is convex
    // (i.e. all points are part of the convex hull)
    std::vector<cv::Point2f> convex_hull;
    GrahamScanner::grahamScan(scene_corners, convex_hull);
    if(convex_hull.size() < scene_corners.size())
    {
      return false;
    }
    // Success
    return true;
  }
  else
  {
    return false;
  }
}
