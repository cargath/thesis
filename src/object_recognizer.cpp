/**
 * @author Carsten Könemann
 */

#include <thesis/object_recognizer.h>

#include <thesis/config.h>

typedef std::vector<cv::DMatch> DMatches;

ObjectRecognizer::ObjectRecognizer()
{
  // Default constructor
}

ObjectRecognizer::~ObjectRecognizer()
{
  // Default destructor
}

ObjectRecognizer::ProcessedSample ObjectRecognizer::processSample(const cv::Mat& image, const std::string& id)
{
  // Make new struct manually
  ProcessedSample sample;
  sample.width  = image.cols;
  sample.height = image.rows;
  sample.id     = id;
  // Detect keypoints
  feature_detector.detect(image, sample.keypoints);
  // Compute descriptors
  descriptor_extractor.compute(image, sample.keypoints, sample.descriptors);
  //
  return sample;
}

void ObjectRecognizer::detectObject(const std::vector<cv::KeyPoint>& keypoints,
                                    const cv::Mat& descriptors,
                                    const ObjectRecognizer::ProcessedSample& object,
                                    ObjectRecognizer::Finding& finding,
                                    cv::Mat* debug_image)
{
  if(descriptors.empty()) return;
  // Compute matches
  std::vector<cv::DMatch> matches;
  descriptor_matcher.match(object.descriptors, descriptors, matches);
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
      object_in_scene.push_back(object.keypoints[matches_filtered[i].queryIdx].pt);
      scene.push_back(keypoints[matches_filtered[i].trainIdx].pt);
    }
    // Compute homography
    cv::Mat homography = cv::findHomography(object_in_scene, scene, CV_RANSAC);
    // Compute perspective transform
    std::vector<cv::Point2f> object_corners = std::vector<cv::Point2f>(4),
                             scene_corners  = std::vector<cv::Point2f>(4);
    object_corners[0] = cvPoint(           0,             0);
    object_corners[1] = cvPoint(object.width,             0);
    object_corners[2] = cvPoint(object.width, object.height);
    object_corners[3] = cvPoint(           0, object.height);
    cv::perspectiveTransform(object_corners, scene_corners, homography);
    // Add object to output
    finding.id           = object.id;
    finding.image_points = scene_corners;
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

void ObjectRecognizer::detectObject(const cv::Mat& image,
                                    const ObjectRecognizer::ProcessedSample& object,
                                    ObjectRecognizer::Finding& finding,
                                    cv::Mat* debug_image)
{
  // Extract feature points
  std::vector<cv::KeyPoint> keypoints;
  feature_detector.detect(image, keypoints);
  // Compute descriptors
  cv::Mat descriptors;
  descriptor_extractor.compute(image, keypoints, descriptors);
  // Detect object
  detectObject(keypoints, descriptors, object, finding, debug_image);
  // Create debug image
  if(debug_image)
  {
    cv::drawKeypoints(*debug_image, keypoints, *debug_image, BLUE);
  }
}

void ObjectRecognizer::detectObjects(const cv::Mat& image,
                                     const std::vector<ObjectRecognizer::ProcessedSample>& objects,
                                     std::vector<ObjectRecognizer::Finding>& findings,
                                     cv::Mat* debug_image)
{
  // Extract feature points
  std::vector<cv::KeyPoint> keypoints;
  feature_detector.detect(image, keypoints);
  // Compute descriptors
  cv::Mat descriptors;
  descriptor_extractor.compute(image, keypoints, descriptors);
  // Detect objects
  for(size_t i = 0; i < objects.size(); i++)
  {
    Finding finding;
    detectObject(keypoints, descriptors, objects[i], finding, debug_image);
    findings.push_back(finding);
  }
  // Create debug image
  if(debug_image)
  {
    cv::drawKeypoints(*debug_image, keypoints, *debug_image, BLUE);
  }
}

void ObjectRecognizer::detectObjectsClustered(const cv::Mat& image,
                                              const std::vector<ProcessedSample>& objects,
                                              std::vector<Finding>& findings,
                                              cv::Mat* debug_image)
{
  // Extract feature points
  std::vector<cv::KeyPoint> keypoints;
  feature_detector.detect(image, keypoints);
  // Cluster feature points
  std::vector<std::vector<cv::KeyPoint> > keypoints_clustered;
  keypoints_clustered = dbscanner.scan(keypoints, DBSCAN_MIN_POINTS, DBSCAN_MAX_DISTANCE);
  // Compute descriptors
  std::vector<cv::Mat> descriptors;
  for(size_t i = 0; i < keypoints_clustered.size(); i++)
  {
    cv::Mat next_descriptors;
    descriptor_extractor.compute(image, keypoints_clustered[i], next_descriptors);
    descriptors.push_back(next_descriptors);
  }
  // Match descriptors
  std::vector<std::vector<cv::DMatch> > matches;
  std::vector<DMatchCluster> matches_clustered;
  for(size_t i = 0; i < descriptors.size(); i++)
  {
    for(size_t j = i+1; j < descriptors.size(); j++)
    {
      // Compute matches
      std::vector<cv::DMatch> next_matches;
      descriptor_matcher.match(descriptors[i], descriptors[j], next_matches);
      matches.push_back(next_matches);
      // Filter matches by distance
      double match_min_distance = 100.0,
             match_max_distance =   0.0;
      // TODO
      // Associate matches with clusters of keypoints
      matches_clustered.push_back(DMatchCluster(matches.size()-1, i, j));
    }
  }
  // Locate objects
  // TODO
  // Create debug image
  if(debug_image)
  {
    // Feature points
    for(size_t i = 0; i < keypoints_clustered.size(); i++)
    {
      int c = 255/keypoints_clustered.size() * i;
      cv::Scalar color = cv::Scalar(255-c, 0, c);
      cv::drawKeypoints(*debug_image, keypoints_clustered[i], *debug_image, color);
    }
    // TODO: Matches
    /*for(size_t i = 0; i < matches_clustered.size(); i++)
    {
      drawMatches(debug_img,
                  *matches_clustered[i].matches,
                  *matches_clustered[i].queryCluster,
                  *matches_clustered[i].trainCluster,
                  GREEN);
    }*/
  }
}

void ObjectRecognizer::drawMatch(cv::Mat& img,
                                 const cv::DMatch& match,
                                 const std::vector<cv::KeyPoint>& a,
                                 const std::vector<cv::KeyPoint>& b,
                                 const cv::Scalar& color,
                                 int thickness,
                                 int line_type)
{
  cv::Point s = cv::Point(a[match.queryIdx].pt.x, a[match.queryIdx].pt.y),
            e = cv::Point(b[match.trainIdx].pt.x, b[match.trainIdx].pt.y);
  cv::line(img, s, e, color, thickness, line_type);
}

void ObjectRecognizer::drawMatches(cv::Mat& img,
                                   const std::vector<cv::DMatch>& matches,
                                   const std::vector<cv::KeyPoint>& a,
                                   const std::vector<cv::KeyPoint>& b,
                                   const cv::Scalar& color,
                                   int thickness,
                                   int line_type)
{
  for(size_t i = 0; i < matches.size(); i++)
  {
    drawMatch(img, matches[i], a, b, color, thickness, line_type);
  }
}
