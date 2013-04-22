/**
 * Defines a bunch of parameters as constants.
 * - Makes the code more readable
 * - Easy tweaking of parameters
 *
 * @author Carsten Könemann
 */
#ifndef __CONFIG__
#define __CONFIG__

#include <opencv2/opencv.hpp>

//
static const std::string  CAMERA_FRAME              = "/head_mount_kinect_rgb_optical_frame";
static const std::string  MAP_FRAME                 = "/map";

// Canny edge detection
static const int          CANNY_LOW_THRESHOLD       = 50;

// Hough line detection
static const double       HOUGH_ANGLE_RESOLUTION    = CV_PI/180.0;
static const int          HOUGH_THRESHOLD           = 80;
static const double       HOUGH_MIN_LINE_LENGTH     = 30.0;
static const double       HOUGH_MAX_LINE_GAP        = 10.0;

// Line filtering
static const double       LINE_HORIZONTAL_THRESHOLD = 45.0;
static const unsigned int LINE_MAX_NOF              = 10;

// KeyPoint clustering
static const unsigned int DBSCAN_MIN_POINTS         = 5;
static const unsigned int DBSCAN_MAX_DISTANCE       = 25;

#endif //__CONFIG__
