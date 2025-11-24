#include "edge_detection.h"

cv::Mat binary_array(const cv::Mat& array, cv::Vec2f thresh, int value) {
    cv::Mat binary;

    if (value == 0) {
        binary = cv::Mat::ones(array.size(), array.type());
    } else {
        binary = cv::Mat::zeros(array.size(), array.type());
    }

    // Threshold aralığında kalan piksellerin maskesini oluştur
    cv::Mat mask;
    cv::inRange(array, thresh[0], thresh[1], mask);

    // mask = 255 olan yerleri value ile doldur
    binary.setTo(value, mask);

    return binary;
}

cv::Mat blur_gaussian(const cv::Mat& image, int ksize) {
    cv::Mat blurred_image;
    // Gaussian blur uygulama
    cv::GaussianBlur(image, blurred_image, cv::Size(ksize, ksize), 0);
    return blurred_image;
}

cv::Mat mag_thresh(const cv::Mat& image, int sobel_kernel, cv::Vec2f thresh) {
    // Apply Sobel in X and Y directions
    cv::Mat sobelx = cv::abs(sobel(image, "x", sobel_kernel));
    cv::Mat sobely = cv::abs(sobel(image, "y", sobel_kernel));

    // Compute magnitude
    cv::Mat mag;
    cv::magnitude(sobelx, sobely, mag);  // Calculate magnitude

    // Apply binary thresholding
    return binary_array(mag, thresh);
}

cv::Mat sobel(const cv::Mat& image, const std::string& orient, int sobel_kernel) {
    cv::Mat sobel_image;

    if (orient == "x") {
        // X yönü için Sobel filtre uygulama
        cv::Sobel(image, sobel_image, CV_64F, 1, 0, sobel_kernel);
    } else if (orient == "y") {
        // Y yönü için Sobel filtre uygulama
        cv::Sobel(image, sobel_image, CV_64F, 0, 1, sobel_kernel);
    }

    return sobel_image;
}

cv::Mat threshold(cv::Mat& image, cv::Scalar thresh, int thresh_type) {
    cv::Mat output;
    // Thresholding işlemi: image = giriş görüntüsü, thresh[0] = min, thresh[1] = max, thresh_type = eşik tipi
    cv::threshold(image, output, thresh[0], thresh[1], thresh_type);
    return output;
}

void printImageArray(const cv::Mat& image){
    for (int y = 0; y < image.rows; y++) {
        for (int x = 0; x < image.cols; x++) {
            uchar pixel_value = image.at<uchar>(y, x);
            std::cout << (int)pixel_value << " ";
        }
        std::cout << std::endl;
    }
}