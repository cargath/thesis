/**
 * @author Carsten Könemann
 */

#include <thesis/math2d.h>

bool intersect(cv::Vec4i a, cv::Vec4i b, cv::Point& intersection)
{
  cv::Point da = cv::Point(a[2] - a[0], a[3] - a[1]);
  cv::Point db = cv::Point(b[2] - b[0], b[3] - b[1]);
  double d = cross2f(da, db);
  // Lines are not parallel, compute intersection
  if(d != 0)
  {
    double temp = cross2f(cv::Point(b[0] - a[0], b[1] - a[1]), da) / d;
    intersection.x = db.x*temp + b[0];
    intersection.y = db.y*temp + b[1];
    return true;
  }
  // Lines are parallel, they do not intersect
  else
  {
    return false;
  }
}
