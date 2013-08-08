/**
 * @author Carsten Könemann
 */
#ifndef __MATH3D__
#define __MATH3D__

#include <thesis/math2d.h>

#include <tf/tf.h>

/**
 * Constants.
 */

// Identity
static const tf::Quaternion IDENTITY_QUATERNION = tf::createIdentityQuaternion();


/**
 * Quaternions.
 */

inline bool isnan(geometry_msgs::Quaternion q)
{
  return isnan(q.x * q.y * q.z * q.w);
}

inline void normalize_quaternion_msg(geometry_msgs::Quaternion& q)
{
  double m = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  q.x = q.x / m;
  q.y = q.y / m;
  q.z = q.z / m;
  q.w = q.w / m;
}


/**
 * Points.
 */

inline bool isnan(geometry_msgs::Point p)
{
  return isnan(p.x * p.y * p.z);
}

inline bool isnan(cv::Point3f p)
{
  return isnan(p.x * p.y * p.z);
}

inline cv::Point3f ros2cv3f(geometry_msgs::Point p)
{
  cv::Point3f p_;
  p_.x = p.x;
  p_.y = p.y;
  p_.z = p.z;
  return p_;
}

inline geometry_msgs::Point cv2ros3f(cv::Point3f p)
{
  geometry_msgs::Point p_;
  p_.x = p.x;
  p_.y = p.y;
  p_.z = p.z;
  return p_;
}

inline float dot3f(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return (a.x*b.x + a.y*b.y + a.z*b.z);
}

inline float dot3f(cv::Point3f a, cv::Point3f b)
{
  return (a.x*b.x + a.y*b.y + a.z*b.z);
}

inline geometry_msgs::Point cross3f(geometry_msgs::Point a, geometry_msgs::Point b)
{
  geometry_msgs::Point c;
  c.x =    a.y * b.z - a.z * b.y;
  c.y = - (b.z * a.x - b.x * a.z);
  c.z =    a.x * b.y - a.y * b.x;
  return c;
}

inline cv::Point3f cross3f(cv::Point3f a, cv::Point3f b)
{
  cv::Point3f c;
  c.x =    a.y * b.z - a.z * b.y;
  c.y = - (b.z * a.x - b.x * a.z);
  c.z =    a.x * b.y - a.y * b.x;
  return c;
}

inline float mag3f(geometry_msgs::Point p)
{
  return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

inline float mag3f(cv::Point3f p)
{
  return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

inline float dist3f(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return mag3f(ros2cv3f(b) - ros2cv3f(a));
}

inline float dist3f(cv::Point3f a, cv::Point3f b)
{
  return mag3f(b - a);
}

inline geometry_msgs::Point norm3f(geometry_msgs::Point p)
{
  geometry_msgs::Point n;
  float mag = mag3f(p);
  n.x = p.x / mag;
  n.y = p.y / mag;
  n.z = p.z / mag;
  return n;
}

inline cv::Point3f norm3f(cv::Point3f p)
{
  cv::Point3f n;
  float mag = mag3f(p);
  n.x = p.x / mag;
  n.y = p.y / mag;
  n.z = p.z / mag;
  return n;
}

inline geometry_msgs::Point xyz2ypr(const geometry_msgs::Point& xyz)
{
  geometry_msgs::Point ypr;
  // Yaw, rotation around y-axis
  ypr.x = angle2f(xyz.x, xyz.z);
  // Pitch, rotation around x-axis
  ypr.y = angle2f(xyz.z, xyz.y);
  // Roll, rotation around z-axis
  ypr.z = angle2f(xyz.x, xyz.y);
  //
  return ypr;
}

inline cv::Point3f xyz2ypr(const cv::Point3f& xyz)
{
  cv::Point3f ypr;
  // Yaw, rotation around y-axis
  ypr.x = angle2f(xyz.x, xyz.z);
  // Pitch, rotation around x-axis
  ypr.y = angle2f(xyz.z, xyz.y);
  // Roll, rotation around z-axis
  ypr.z = angle2f(xyz.x, xyz.y);
  //
  return ypr;
}

inline float angle3f(geometry_msgs::Point dir1, geometry_msgs::Point dir2)
{
  return acos(dot3f(dir1, dir2) / (mag3f(dir1) * mag3f(dir2)));
}

inline float angle3f(cv::Point3f dir1, cv::Point3f dir2)
{
  return acos(dot3f(dir1, dir2) / (mag3f(dir1) * mag3f(dir2)));
}

inline cv::Point3f centroid3f(const std::vector<cv::Point3f>& v)
{
  cv::Point3f c;
  for(size_t i = 0; i < v.size(); i++)
  {
    c.x += v[i].x;
    c.y += v[i].y;
    c.z += v[i].z;
  }
  c.x /= v.size();
  c.y /= v.size();
  c.z /= v.size();
  return c;
}

#endif //__MATH3D__
