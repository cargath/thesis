/**
 * This file was part of RGBDSLAM by Felix Endres.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Created by Felix Endres,
 *         slightly modified by Carsten Könemann.
 */

#ifdef USE_SIFT_GPU

#include <thesis/sift_gpu_wrapper.h>

#include <ros/ros.h>

#include <GL/gl.h>

#include <iostream>
#include <stdio.h>

using namespace cv;

SiftGPUWrapper* SiftGPUWrapper::instance = NULL;

SiftGPUWrapper::SiftGPUWrapper()
{
  data = NULL;
  error = false;
  data_size = 0;
  siftgpu = new SiftGPU();
  
  char method[] = {"-glsl"};

  int max_features = 600;
  char max_feat_char[10];
  sprintf(max_feat_char, "%d", max_features);

  char subpixelKey[] = {"-s"};
  char subpixelValue[] = {"0"};
  char max_flag[] = {"-tc2"};

  // nothing but errors
  char verbosity[] = {"-v"};
  // nothing but errors
  char verbosity_val[] = {"0"};

  char first_octave[] = {"-fo"};
  char first_octave_val[] = {"0"};
  
  char* argv[] =
  {
    method, subpixelKey, subpixelValue,
    max_flag, max_feat_char, first_octave,
    first_octave_val, verbosity, verbosity_val
  };
  
  siftgpu->ParseParam(9, argv);

  if(siftgpu->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
  {
    ROS_ERROR("SiftGPUWrapper: ");
    ROS_ERROR("  Can't create OpenGL context! SiftGPU cannot be used.");
    error = true;
  }

  isMatcherInitialized = false;
}

SiftGPUWrapper::~SiftGPUWrapper()
{
  // SiftGPU extractor
  delete siftgpu;
  // SiftGPU matcher
  delete matcher;
  // Singleton instance of this class
  delete instance;
  // Image as texture
  free(data);
}

/*bool SiftGPUWrapper::init(const int max_nof_keypoints,
                          const double knn_1to2_ratio)
{
  // Extractor
  data = NULL;
  error = false;
  data_size = 0;
  
  siftgpu = new SiftGPU();
  
  char method[]           = {"-glsl"};

  char subpixelKey[]      = {"-s"};
  char subpixelValue[]    = {"0"};
  char max_flag[]         = {"-tc2"};

  int  max_features = 600;
  char max_feat_char[10];
  //
  sprintf(max_feat_char, "%d", max_features);

  char first_octave[]     = {"-fo"};
  char first_octave_val[] = {"0"};
  
  // nothing but errors
  char verbosity[]        = {"-v"};
  // nothing but errors
  char verbosity_val[]    = {"0"};
  
  char* argv[] =
  {
    method,
    subpixelKey,
    subpixelValue,
    max_flag,
    max_feat_char,
    first_octave,
    first_octave_val,
    verbosity,
    verbosity_val
  };
  
  siftgpu->ParseParam(9, argv);

  if(siftgpu->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
  {
    ROS_ERROR("SiftGPUWrapper: ");
    ROS_ERROR("  Can't create OpenGL context! SiftGPU cannot be used.");
    error = true;
  }

  // Matcher
  
}*/

SiftGPUWrapper* SiftGPUWrapper::getInstance()
{
  if(instance == NULL)
  {
    ROS_INFO("SiftGPU Wrapper: Creating instance.");
    instance = new SiftGPUWrapper();
  }
  return instance;
}

void SiftGPUWrapper::compute(const cv::Mat& image,
                             const std::vector<cv::KeyPoint>& keypoints,
                             std::vector<float>& descriptors)
{
  // TODO
}

void SiftGPUWrapper::detect(const cv::Mat& image,
                            std::vector<cv::KeyPoint>& keypoints,
                            std::vector<float>& descriptors)
{
  if(error)
  {
    keypoints.clear();
    ROS_FATAL("SiftGPUWrapper: ");
    ROS_FATAL("  SiftGPU cannot be used. Detection of keypoints failed.");
    return;
  }

  // Allocate memory to store image data
  // No need to allocate memory again for multiple images of the same size
  int current_image_size = image.rows * image.cols;
  if(current_image_size != data_size)
  {
    data_size = current_image_size;
    free(data);
    data = (unsigned char*) malloc(data_size);
  }
  
  // Convert OpenCV image to SiftGPU image
  cvMatToSiftGPU(image, data);

  int num_features = 0;
  SiftGPU::SiftKeypoint* keys = 0;

  if(siftgpu->RunSIFT(image.cols, image.rows, data, GL_LUMINANCE, GL_UNSIGNED_BYTE))
  {
    num_features = siftgpu->GetFeatureNum();
    ROS_DEBUG("SiftGPUWrapper: #features found: %i", num_features);
    keys = new SiftGPU::SiftKeypoint[num_features];
    descriptors.resize(128 * num_features);
    //descriptors = new float[128 * num_features];
    siftgpu->GetFeatureVector(&keys[0], &descriptors[0]);
  }
  else
  {
    ROS_WARN("SiftGPUWrapper: SiftGPU->RunSIFT() failed!");
  }

  // Copy to OpenCV structure
  keypoints.clear();
  for(int i = 0; i < num_features; ++i)
  {
    // 6x scale is the conversion to pixels
    // (according to Changchang Wu, the author of SiftGPU)
    KeyPoint key(keys[i].x, keys[i].y, 6.0 * keys[i].s, keys[i].o);
    keypoints.push_back(key);
  }
}

bool SiftGPUWrapper::match(const std::vector<float>& descriptors1,
                           const std::vector<float>& descriptors2,
                           std::vector<cv::DMatch>& matches)
{
  if(!isMatcherInitialized)
  {
    initializeMatcher();
  }

  float sumDistances = 0;

  int num1 = descriptors1.size() / 128,
      num2 = descriptors2.size() / 128;

  matcher->SetDescriptors(0, num1, &descriptors1[0]);
  matcher->SetDescriptors(1, num2, &descriptors2[0]);

  int (*match_buf)[2] = new int[num1][2];
  int number = matcher->GetSiftMatch(num1, match_buf, 0.9, 0.9);

  cv::DMatch match;
  int counter = 0;

  matches.clear();
  
  for(int i = 0; i < number; i++)
  {
    match.queryIdx = match_buf[i][0];
    match.trainIdx = match_buf[i][1];

    // Only use matches with indices != 0
    // (OpenGL context problem may sometimes happen)
    if(match.queryIdx == 0 || match.trainIdx == 0)
    {
      counter++;
    }

    if(counter > 0.5 * number)
    {
      matches.clear();
      sumDistances = 0;
      ROS_ERROR("SiftGPUWrapper: Matches bad due to context error.");
      return false;
    }

    float sum = 0;
    for(int j = 0; j < 128; j++)
    {
      float a = descriptors1[match.queryIdx * 128 + j] - descriptors2[match.trainIdx * 128 + j];
      sum += a * a;
    }

    match.distance = sqrt(sum);
    sumDistances += match.distance;
    
    ROS_DEBUG("SiftGPUWrapper: ");
    ROS_DEBUG("  Matched Features %d and %d with distance of %f. Sum: %f",
              match.queryIdx,
              match.trainIdx,
              match.distance,
              sumDistances);
              
    matches.push_back(match);
  }

  ROS_DEBUG("SiftGPUWrapper: #matches found: %lo", matches.size());

  delete[] match_buf;

  return true;
}

void SiftGPUWrapper::initializeMatcher()
{
  matcher = CreateNewSiftMatchGPU(4096);
  if(!matcher->VerifyContextGL())
  {
    ROS_FATAL("SiftGPUWrapper: ");
    ROS_FATAL("  Can't create OpenGL context! SiftGPU Matcher cannot be used.");
    ROS_FATAL("  You probably have no OpenGL support.");
    ROS_FATAL("  Try using a different type of matcher, e.g. FLANN.");
    error = true;
    return;
  }
  ROS_INFO("SiftGPUWrapper: SiftGPU matcher initialized successfully.");
  isMatcherInitialized = true;
}

void SiftGPUWrapper::cvMatToSiftGPU(const Mat& image, unsigned char* siftImage)
{
  Mat tmp;
  image.convertTo(tmp, CV_8U);
  for(int y = 0; y < tmp.rows; ++y)
  {
    for(int x = 0; x < tmp.cols; ++x)
    {
      siftImage[y * tmp.cols + x] = tmp.at<unsigned char>(y, x);
    }
  }
}

#endif // USE_SIFT_GPU
