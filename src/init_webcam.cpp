/**
 * @author Carsten Könemann
 */

// This is a ROS project
#include <ros/ros.h>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// We are going to publish messages of these types
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// Debug image window
static const std::string DEBUG_IMAGE_WINDOW = "Debug Image";

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "thesis_webcam");
  ros::NodeHandle nh_private("~");
  // Initialize publishers
  ros::Publisher rgb_publisher = nh_private.advertise<sensor_msgs::Image>("rgb", 1000);;
  ros::Publisher depth_publisher = nh_private.advertise<sensor_msgs::Image>("depth", 1000);;
  ros::Publisher camera_info_publisher = nh_private.advertise<sensor_msgs::CameraInfo>("camera_info", 1000);;
  // Initialize webcam
  CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
  if(!capture)
  {
    ROS_ERROR("Capture is NULL.");
    return 1;
  }
  cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH,  640);
  cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
  // Create debug image window
  cv::namedWindow(DEBUG_IMAGE_WINDOW, CV_WINDOW_AUTOSIZE);
  // Show the image captured from the camera in the window and repeat
  while(true)
  {
    // Get next frame
    IplImage* frame = cvQueryFrame(capture);
    if(!frame)
    {
      ROS_ERROR("Frame is NULL.");
      break;
    }
    // Convert OpenCV image to ROS image message
    sensor_msgs::Image ros_message;
    cv_bridge::CvImage cv_image;
    cv_image.encoding = sensor_msgs::image_encodings::MONO8;
    cv_image.image    = cv::Mat(frame);
    cv_image.toImageMsg(ros_message);
    // Publish message
    rgb_publisher.publish(ros_message);
    // Show debug image
    cv::imshow(DEBUG_IMAGE_WINDOW, cv_image.image);
    cv::waitKey(3);
  }
  // Free memory
  cvReleaseCapture(&capture);
  cv::destroyWindow(DEBUG_IMAGE_WINDOW);
  // Exit with success
  return 0;
}
