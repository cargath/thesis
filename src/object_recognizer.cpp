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

void ObjectRecognizer::detectObject(const std::vector<cv::KeyPoint>& keypoints,
                                    const cv::Mat& descriptors,
                                    ObjectDatabase::SampleObject& object,
                                    std::vector<Finding>& findings,
                                    cv::Mat* debug_img)
{
  // Compute matches
  std::vector<cv::DMatch> matches;
  descriptor_matcher.match(object.desc, descriptors, matches);
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
    object_corners[0] = cvPoint(              0,               0);
    object_corners[1] = cvPoint(object.img.cols,               0);
    object_corners[2] = cvPoint(object.img.cols, object.img.rows);
    object_corners[3] = cvPoint(              0, object.img.rows);
    cv::perspectiveTransform(object_corners, scene_corners, homography);
    // Add object to output
    findings.push_back(Finding(object.name, scene_corners));
    // Create debug image
    if(debug_img)
    {
      line(*debug_img, scene_corners[0], scene_corners[1], RED);
      line(*debug_img, scene_corners[1], scene_corners[2], RED);
      line(*debug_img, scene_corners[2], scene_corners[3], RED);
      line(*debug_img, scene_corners[3], scene_corners[0], RED);
    }
  }
}

void ObjectRecognizer::detectObjects(const cv::Mat& img,
                                     ObjectDatabase& database,
                                     std::vector<Finding>& findings,
                                     cv::Mat* debug_img)
{
  // Extract feature points
  std::vector<cv::KeyPoint> keypoints;
  feature_detector.detect(img, keypoints);
  // Compute descriptors
  cv::Mat descriptors;
  descriptor_extractor.compute(img, keypoints, descriptors);
  // Detect objects
  for(size_t i = 0; i < database.objects.size(); i++)
  {
    detectObject(keypoints, descriptors, database.objects[i], findings, debug_img);
  }
  // Create debug image
  if(debug_img)
  {
    cv::drawKeypoints(*debug_img, keypoints, *debug_img, BLUE);
  }
}

void ObjectRecognizer::detectObjectsClustered(const cv::Mat& img,
                                              ObjectDatabase& database,
                                              std::vector<Finding>& findings,
                                              cv::Mat* debug_img)
{
  // Extract feature points
  std::vector<cv::KeyPoint> keypoints;
  feature_detector.detect(img, keypoints);
  // Cluster feature points
  std::vector<std::vector<cv::KeyPoint> > keypoints_clustered;
  keypoints_clustered = dbscanner.scan(keypoints, DBSCAN_MIN_POINTS, DBSCAN_MAX_DISTANCE);
  // Compute descriptors
  std::vector<cv::Mat> descriptors;
  for(size_t i = 0; i < keypoints_clustered.size(); i++)
  {
    cv::Mat next_descriptors;
    descriptor_extractor.compute(img, keypoints_clustered[i], next_descriptors);
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
  if(debug_img)
  {
    // Feature points
    for(size_t i = 0; i < keypoints_clustered.size(); i++)
    {
      int c = 255/keypoints_clustered.size() * i;
      cv::Scalar color = cv::Scalar(255-c, 0, c);
      cv::drawKeypoints(*debug_img, keypoints_clustered[i], *debug_img, color);
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

void ObjectRecognizer::detectShelves(const cv::Mat& img, cv::Mat* debug_img)
{
  // Edge detection
  cv::Mat img_edges;
  cv::Canny(img, img_edges, CANNY_LOW_THRESHOLD, CANNY_LOW_THRESHOLD*3, 3);
  // Line detection
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(img_edges, lines, 1, HOUGH_ANGLE_RESOLUTION, HOUGH_THRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);
  // Filter lines; We are interested in "horizontal" lines only
  // TODO: use 3D transforms to determine which lines are actually horizontal
  std::vector<cv::Vec4i> horizontal_lines;
  for(size_t i = 0; i < lines.size(); i++)
  {
    if(abs(angle(lines[i])) <= LINE_HORIZONTAL_THRESHOLD)
    {
      horizontal_lines.push_back(lines[i]);
    }
  }
  // Filter lines; We are interested in the LINE_MAX_NOF longest lines only
  std::vector<cv::Vec4i> longest_lines;
  std::sort(horizontal_lines.begin(), horizontal_lines.end(), CvVec4iComparator());
  for(size_t i = 0; i < horizontal_lines.size() && i <= LINE_MAX_NOF; i++)
  {
    longest_lines.push_back(horizontal_lines[i]);
  }
  // Determine where the shelves run
  // TODO
  // Create debug image
  if(debug_img)
  {
    for(size_t i = 0; i < longest_lines.size(); i++)
    {
      cv::Point a = cv::Point(longest_lines[i][0], longest_lines[i][1]),
                b = cv::Point(longest_lines[i][2], longest_lines[i][3]);
      cv::line(*debug_img, a, b, RED, 2, 8);
    }
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
