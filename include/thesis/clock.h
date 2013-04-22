/**
 * A simple "stopwatch" to measure code execution time.
 * These macros utilize OpenCVs cvGetTickCount() and cvGetTickFrequency().
 *
 * @author Carsten Könemann
 */
#ifndef __CLOCK__
#define __CLOCK__

#include <opencv2/opencv.hpp>

#define CLOCK_START(n)    int64 clock_##n = cvGetTickCount()
#define CLOCK_STOP(n)     std::cout << #n << ": " << (double) (cvGetTickCount() - clock_##n) / (cvGetTickFrequency() * 1000.0) << std::endl
#define CLOCK_STOPM(n, m) std::cout << #m << ": " << (double) (cvGetTickCount() - clock_##n) / (cvGetTickFrequency() * 1000.0) << std::endl

#endif //__CLOCK__
