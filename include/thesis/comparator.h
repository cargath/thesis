/**
 * Extension for third-party objects with undefined comparison operators.
 *
 * @author Carsten Könemann
 */
#ifndef __COMPARATOR__
#define __COMPARATOR__

#include <thesis/math2d.h>

struct CvKeyPointComparator
{
  bool operator() (const cv::KeyPoint &a, const cv::KeyPoint &b)
  {
    if(a.pt.x != b.pt.x) return a.pt.x < b.pt.x;
    else return a.pt.y < b.pt.y;
  }
};

struct CvVec4iComparator
{
  bool operator() (cv::Vec4i const &a, cv::Vec4i const &b) const
  {
    return length(a) > length(b);
  }
};

#endif //__COMPARATOR__
