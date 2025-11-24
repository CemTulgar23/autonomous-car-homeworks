#ifndef EDGE_DETECTION_H
#define EDGE_DETECTION_H

#include <opencv2/opencv.hpp>

cv::Mat binary_array(const cv::Mat& array, cv::Vec2f thresh, int value = 0);

cv::Mat blur_gaussian(const cv::Mat& image, int ksize = 3);

cv::Mat mag_thresh(const cv::Mat& image, int sobel_kernel = 3, cv::Vec2f thresh = cv::Vec2f(0, 255));

cv::Mat sobel(const cv::Mat& image, const std::string& orient = "x", int sobel_kernel = 3);

cv::Mat threshold(cv::Mat& image, cv::Scalar thresh = cv::Scalar(128, 255), int thresh_type = cv::THRESH_BINARY);

void printImageArray(const cv::Mat& image);

#endif // EDGE_DETECTION_H