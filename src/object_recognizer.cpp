/**
 * @author Carsten Könemann
 */

#include <thesis/object_recognizer.h>

typedef std::vector<cv::DMatch> DMatches;

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
}

void ObjectRecognizer::recognize(const cv::Mat& camera_image,
                                 const ImageInfo& sample_info,
                                 std::vector<cv::Point2f>& object_points,
                                 cv::Mat* debug_image)
{
  ImageInfo cam_img_info;
  getImageInfo(camera_image, cam_img_info);
  recognize(cam_img_info, sample_info, object_points, debug_image);
}

void ObjectRecognizer::recognize(const ImageInfo& cam_img_info,
                                 const ImageInfo& sample_info,
                                 std::vector<cv::Point2f>& object_points,
                                 cv::Mat* debug_image)
{
  if(cam_img_info.descriptors.empty()) return;
  // Compute matches
  std::vector<cv::DMatch> matches;
  descriptor_matcher.match(sample_info.descriptors, cam_img_info.descriptors, matches);
  std::cout << matches.size() << std::endl;
  // Filter matches by distance
  double min_distance = 100.0,
         max_distance =   0.0;
  for(size_t i = 0; i < matches.size(); i++)
  {
    if(matches[i].distance < min_distance)
    {
      min_distance = matches[i].distance;
    }
    if(matches[i].distance > max_distance)
    {
      max_distance = matches[i].distance;
    }
  }
  std::cout << "min_distance: " << min_distance << std::endl;
  std::vector<cv::DMatch> matches_filtered;
  for(size_t i = 0; i < matches.size(); i++)
  {
    if(matches[i].distance < 2*min_distance)
    {
      matches_filtered.push_back(matches[i]);
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
    // Add recognized object points to output
    object_points = scene_corners;
    // Create debug image
    if(debug_image)
    {
      line(*debug_image, scene_corners[0], scene_corners[1], RED);
      line(*debug_image, scene_corners[1], scene_corners[2], RED);
      line(*debug_image, scene_corners[2], scene_corners[3], RED);
      line(*debug_image, scene_corners[3], scene_corners[0], RED);
    }
  }
}
