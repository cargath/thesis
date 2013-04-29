/**
 * Defines a bunch of parameters as constants.
 * - Makes the code more readable
 * - Easy tweaking of parameters
 *
 * @author Carsten Könemann
 */
#ifndef __CONFIG__
#define __CONFIG__

// TF
static const std::string   CAMERA_FRAME = "/head_mount_kinect_rgb_optical_frame";
static const std::string   MAP_FRAME    = "/map";
static const ros::Duration TF_TIMEOUT   = ros::Duration(5.0);

// KeyPoint clustering
static const unsigned int DBSCAN_MIN_POINTS   = 5;
static const unsigned int DBSCAN_MAX_DISTANCE = 25;

#endif //__CONFIG__
