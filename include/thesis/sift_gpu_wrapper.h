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
#ifndef __SIFT_GPU_WRAPPER__
#define __SIFT_GPU_WRAPPER__

#ifdef USE_SIFT_GPU

#include <opencv2/features2d/features2d.hpp>
#include <opencv/cv.h>

#include <SiftGPU/SiftGPU.h>

/**
 * @brief Interface for SiftGPU.
 *
 * The class is used as an interface to SiftGPU.
 * This is a singleton class.
 */
class SiftGPUWrapper
{
  public:
    /**
     * Destructor
     */
	  virtual ~SiftGPUWrapper();
    
    /**
     * Compute descriptors for existing keypoints.
     *
     * @param image       The image.
	   * @param keypoints   The existing keypoints.
	   * @param descriptors The computed descriptors (output).
     */
    void compute(const cv::Mat& image,
                 const std::vector<cv::KeyPoint>& keypoints,
                 std::vector<float>& descriptors);

	  /**
	   * Detect keypoints and compute their descriptors.
	   *
	   * @param image       The image.
	   * @param keypoints   The detected keypoints (output).
	   * @param descriptors The computed descriptors (output).
	   */
	  void detect(const cv::Mat& image,
	              std::vector<cv::KeyPoint>& keypoints,
	              std::vector<float>& descriptors);

	  /**
	   * Is used for matching two sets of descriptors.
	   *
	   * @param descriptors1 The first descriptor.
	   * @param descriptors2 The second descriptor.
	   * @param matches      Is used to store the matches.
	   *
	   * @return false if there was an error during the matching process,
	   *         true  otherwise.
	   */
	  bool match(const std::vector<float>& descriptors1,
	             const std::vector<float>& descriptors2,
	             std::vector<cv::DMatch>& matches);

	  /**
	   * @return Instance of the singleton class.
	   */
	  static SiftGPUWrapper* getInstance();

  private:
	  /**
	   * Private constructor, because this is a singleton class.
	   */
	  SiftGPUWrapper();

	  void initializeMatcher();

	  /**
	   * Convert a cv::Mat image to a SiftGPU compatible OpenGL texture array.
	   *
	   * @param image     The image.
	   * @param siftImage The transformed image (output).
	   */
	  void cvMatToSiftGPU(const cv::Mat& image, unsigned char* siftImage);

	  /**
	   * For testing purposes: write a .pgm file of the SiftGPU image
	   *
	   * @param fp     A filepointer.
	   * @param data   The image data (e.g. OpenGL texture).
	   * @param width  Width.
	   * @param height Height.
	   */
	  void writePGM(FILE *fp, unsigned char* data, int width, int height);

    // Width of the image constant for Kinect
	  int imageWidth;
	  // Height of the image constant for Kinect
	  int imageHeight;
	  // Image as texture
	  unsigned char* data;

    // Singleton instance
	  static SiftGPUWrapper* instance;
	  // SiftGPU instance
	  SiftGPU* siftgpu;
	  // SiftGPU matcher
	  SiftMatchGPU* matcher;
	  // true if matcher was initialized
	  bool isMatcherInitialized;
	  // true if an error happened
	  bool error;
};

#endif // USE_SIFT_GPU
#endif //__SIFT_GPU_WRAPPER__
