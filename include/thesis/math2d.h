/**
 * @author Carsten Könemann
 */
#ifndef __MATH2D__
#define __MATH2D__

#include <opencv2/opencv.hpp>

// Angular dimensions
static const double RAD = 180.0 / CV_PI;
static const double DEG = CV_PI / 180.0;

/**
 * @return The determinant.
 */
inline float det2f(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c)
{
  return (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
}

/**
 * @return Angle between a->b and the x-axis in degrees (0 - 180).
 */
static inline float angle2f(const cv::Point2f& a, const cv::Point2f& b)
{
  if(b.x == a.x)
  {
    return 90.0f;
  }
  float m = (b.y - a.y) / (b.x - a.x);
  if(m < 0)
  {
    return 180 + atan(m) * RAD;
  }
  return atan(m) * RAD;
}

inline float dot2f(cv::Point2f a, cv::Point2f b)
{
  return (a.x * b.x) + (a.y * b.y);
}

inline float cross2f(cv::Point2f a, cv::Point2f b)
{
  return (a.x * b.y) - (b.x * a.y);
}

inline float distance2f(cv::Point2f a, cv::Point2f b)
{
  return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
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
