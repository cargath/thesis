/**
 * @author Carsten Könemann
 */
#ifndef __MATH2D__
#define __MATH2D__

#include <opencv2/opencv.hpp>

// Angular dimensions
static const double RAD = 180.0/CV_PI;
static const double DEG = CV_PI/180.0;

inline double dot2f(cv::Point2f a, cv::Point2f b)
{
  return (a.x*b.x) + (a.y*b.y);
}

inline double cross2f(cv::Point2f a, cv::Point2f b)
{
  return (a.x*b.y) - (b.x*a.y);
}

inline double angle(cv::Vec4i line)
{
  return atan2(line[3] - line[1], line[2] - line[0]) * RAD;
}

inline double distance(cv::Point2f a, cv::Point2f b)
{
  return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

inline double length(cv::Vec4i line)
{
  return sqrt(pow(line[2] - line[0], 2) + pow(line[3] - line[1], 2));
}

inline float dir2angle(const cv::Point2f& dir)
{
  return atan(dir.y / dir.x);
}

inline float dir2angle(float x, float y)
{
  return atan(y / x);
}

bool intersect(cv::Vec4i a, cv::Vec4i b, cv::Point& intersection);

#endif //__MATH2D__
