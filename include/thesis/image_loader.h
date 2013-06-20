/**
 * @author Carsten Könemann
 */
#ifndef __IMAGE_LOADER__
#define __IMAGE_LOADER__

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

class ImageLoader
{
  public:
    // Default constructor
    ImageLoader();
    // Default destructor
    ~ImageLoader();
    
    // Read a single sample image
    bool load_image(boost::filesystem::path path, cv::Mat& out_image);
    
    // Read all sample images from a directory
    bool load_directory(boost::filesystem::path path,
                        std::vector<cv::Mat>& out_images,
                        std::vector<std::string>* filenames=NULL);
};

#endif //__IMAGE_LOADER__
