/**
 * @author Carsten Könemann
 */

#include <thesis/object_recognizer.h>

#include <thesis/clock.h>
#include <thesis/graham_scanner.h>

#include <opencv2/flann/flann.hpp>

ObjectRecognizer::ObjectRecognizer()
{
  // Default constructor
}

ObjectRecognizer::~ObjectRecognizer()
{
  // Default destructor
}

void ObjectRecognizer::getImageInfo(const cv::Mat& image, ImageInfo& image_info)
{
  image_info.width  = image.cols;
  image_info.height = image.rows;
  // Detect keypoints
  feature_detector.detect(image, image_info.keypoints);
  // Compute descriptors
  descriptor_extractor.compute(image, image_info.keypoints, image_info.descriptors);
  // Train matcher
  std::vector<cv::Mat> descriptor_vector;
  descriptor_vector.push_back(image_info.descriptors);
  image_info.matcher.add(descriptor_vector);
  image_info.matcher.train();
}

bool ObjectRecognizer::recognize(ImageInfo& sample_info,
                                 const cv::Mat& camera_image,
                                 std::vector<cv::Point2f>& object_points)
{
  ImageInfo cam_img_info;
  getImageInfo(camera_image, cam_img_info);
  return recognize(sample_info, cam_img_info, object_points);
}

bool ObjectRecognizer::recognize(ImageInfo& sample_info,
                                 ImageInfo& cam_img_info,
                                 std::vector<cv::Point2f>& object_points)
{
  // Otherwise an OpenCV assertion would fail for images without keypoints
  if(cam_img_info.descriptors.empty() || sample_info.descriptors.empty())
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
    if((scene_corners[0].x < scene_corners[1].x && scene_corners[3].x > scene_corners[2].x)
    || (scene_corners[0].x > scene_corners[1].x && scene_corners[3].x < scene_corners[2].x)
    || (scene_corners[0].y < scene_corners[3].y && scene_corners[1].y > scene_corners[2].y)
    || (scene_corners[0].y > scene_corners[3].y && scene_corners[1].y < scene_corners[2].y))
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
