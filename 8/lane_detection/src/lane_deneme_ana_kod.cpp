#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <algorithm>
#include "edge_detection.h"

class LaneNode : public rclcpp::Node
{
public:
    LaneNode() : Node("lane")
    {
        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/carla/ego_vehicle/rgb_front/image",
            10,
            std::bind(&LaneNode::image_callback, this, std::placeholders::_1));
    }

private:

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // ROS 2 Image mesajını OpenCV formatına çevir
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        width = image.cols;
        height = image.rows;

        // std::cout << "width: " << width << std::endl;
        // std::cout << "height: " << height << std::endl;

        padding = static_cast<int>(0.25 * width);
        desired_roi_points = {
            cv::Point2f(padding, 0),              // Top-left
            cv::Point2f(padding, height),         // Bottom-left
            cv::Point2f(width - padding, height), // Bottom-right
            cv::Point2f(width - padding, 0)       // Top-right
        };

        roi_points = {
            cv::Point2f(width / 2 - 90, height - 200), // Top-left corner
            cv::Point2f(width / 2 - 260, height - 1),  // Bottom-left corner
            cv::Point2f(width / 2 + 260, height - 1),  // Bottom-right corner
            cv::Point2f(width / 2 + 90, height - 200)  // Top-right corner
        };

        orig_image_size = image.size();

        lane_line_markings = get_line_markings(image);

        perspective_transform(image, true);

        calculateHistogram(warped_frame, true);

        std::pair<int, int> peaks = histogram_peak();
        // std::cout << "Left peak: " << peaks.first << ", Right peak: " << peaks.second << std::endl;

        margin = static_cast<int>((1.0 / 12.0) * width); // Window width is +/- margin
        minpix = static_cast<int>((1.0 / 24.0) * width); // Min no. of pixels to recenter window

        get_lane_line_indices_sliding_windows(true);

        /*
        // plot_roi(image, roi_points, true);

        */

        cv::imshow("Image", image);
        //cv::imshow("Lane Line Markings", lane_line_markings);
        cv::waitKey(1);
    }

    cv::Mat get_line_markings(const cv::Mat &frame)
    {
        // 1. BGR'den HLS renk uzayına çevir
        cv::Mat hls;
        cv::cvtColor(frame, hls, cv::COLOR_BGR2HLS);

        // 2. L kanalında threshold ve blur (kenar tespiti için)
        cv::Mat l_channel = hls.clone();
        cv::extractChannel(hls, l_channel, 1); // L channel = channel index 1

        cv::Vec2f l_thresh(120, 255);
        cv::Mat sxbinary = threshold(l_channel, cv::Scalar(l_thresh[0], l_thresh[1]), cv::THRESH_BINARY);
        sxbinary = blur_gaussian(sxbinary, 3);

        // 3. Sobel büyüklüğü threshold (edge detection)
        cv::Vec2f sobel_thresh(110, 255);
        sxbinary = mag_thresh(sxbinary, 3, sobel_thresh);

        // 4. S kanalında binary threshold (saf renkleri bulmak için)
        cv::Mat s_channel = hls.clone();
        cv::extractChannel(hls, s_channel, 2); // S channel = channel index 2

        cv::Vec2f s_thresh(80, 255);
        cv::Mat s_binary = threshold(s_channel, cv::Scalar(s_thresh[0], s_thresh[1]), cv::THRESH_BINARY);

        // 5. BGR görüntüden kırmızı (R) kanalı al
        cv::Mat r_channel;
        cv::extractChannel(frame, r_channel, 2); // R channel = dwindex 2

        cv::Vec2f r_thresh_val(120, 255);
        cv::Mat r_thresh = threshold(r_channel, cv::Scalar(r_thresh_val[0], r_thresh_val[1]), cv::THRESH_BINARY);

        // 6. s_binary ve r_thresh arasında AND işlemi
        cv::Mat rs_binary;
        cv::bitwise_and(s_binary, r_thresh, rs_binary);

        // 7. rs_binary ve sxbinary arasında OR işlemi (final lane maskesi)
        cv::Mat lane_line_markings;
        sxbinary.convertTo(sxbinary, CV_8UC1); // tip uyumu için
        cv::bitwise_or(rs_binary, sxbinary, lane_line_markings);

        return lane_line_markings;
    }

    void plot_roi(cv::Mat frame, const std::vector<cv::Point2f> &roi_points, bool plot = false)
    {
        if (!plot)
            return;

        // ROI noktalarını int'e çevir (çizim için)
        std::vector<cv::Point> roi_points_int;
        for (const auto &pt : roi_points)
            roi_points_int.push_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));

        // Polygon çiz
        std::vector<std::vector<cv::Point>> pts{roi_points_int};
        cv::polylines(frame, pts, true, cv::Scalar(147, 20, 255), 3);

        // Görüntüyü göster
        cv::imshow("ROI Image", frame);
        cv::waitKey(1); // herhangi bir tuşa kadar bekle
    }

    cv::Mat perspective_transform(const cv::Mat &input_frame, bool plot = false)
    {
        // cv::Mat frame = input_frame.empty() ? lane_line_markings : input_frame;
        cv::Mat frame = lane_line_markings;

        // 1. Perspektif dönüşüm matrisini hesapla
        transformation_matrix = cv::getPerspectiveTransform(roi_points, desired_roi_points);

        /*
        std::cout << "Transformation Matrix: " << std::endl;
        for (int i = 0; i < transformation_matrix.rows; ++i)
        {
            for (int j = 0; j < transformation_matrix.cols; ++j)
            {
                std::cout << transformation_matrix.at<double>(i, j) << " ";
            }
            std::cout << std::endl;
        }
        */

        // 2. Ters dönüşüm matrisini hesapla
        inv_transformation_matrix = cv::getPerspectiveTransform(desired_roi_points, roi_points);

        /*
        std::cout << "Inverse Transformation Matrix: " << std::endl;
        for (int i = 0; i < inv_transformation_matrix.rows; ++i)
        {
            for (int j = 0; j < inv_transformation_matrix.cols; ++j)
            {
                std::cout << inv_transformation_matrix.at<double>(i, j) << " ";
            }
            std::cout << std::endl;
        }
        */

        // 3. Perspektif dönüşümü uygula
        cv::warpPerspective(frame, warped_frame, transformation_matrix, orig_image_size, cv::INTER_LINEAR);

        // 4. Görüntüyü binary hale getir (eşikleme)
        // cv::threshold(warped_frame, warped_frame, 127, 255, cv::THRESH_BINARY);

        // 5. Görsel olarak çizmek istiyorsan
        if (plot)
        {
            cv::Mat warped_copy = warped_frame.clone();
            std::vector<std::vector<cv::Point>> poly_points(1);
            for (const auto &pt : desired_roi_points)
                poly_points[0].push_back(pt);

            // Çokgen çiz
            cv::polylines(warped_copy, poly_points, true, cv::Scalar(147, 20, 255), 3);

            // Görüntüyü göster
            cv::imshow("Warped Image", warped_copy);
            cv::waitKey(1);
        }
        return warped_frame;
    }

    std::vector<int> calculateHistogram(cv::Mat frame = cv::Mat(), bool plot = true)
    {
        if (frame.empty())
        {
            frame = warped_frame.clone(); // Daha önceki frame kullanılır
        }

        cv::Point2f left_point = desired_roi_points[0];
        cv::Point2f right_point = desired_roi_points[3];

        // std::cout << "Left x: " << left_point.x << ", Right x: " << right_point.x << std::endl;

        // int histogram_width = right_point.x - left_point.x;

        // Histogramı sıfırla
        histogram = std::vector<int>(width, 0);

        // Alt yarıdan beyaz pikselleri say
        for (int y = 0; y < height; y++)
        {
            for (int x = left_point.x; x < right_point.x + 1; x++)
            {
                unsigned char pixel = frame.at<unsigned char>(y, x);
                // std::cout << "pixel (" << x << ", " << y << "): " << (int)pixel << std::endl;
                if (pixel > 100)
                {
                    histogram[x]++;
                }
            }
        }

        if (plot)
        {
            cv::Mat hist_image(400, width, CV_8UC3, cv::Scalar(0, 0, 0));

            int max_value = *std::max_element(histogram.begin(), histogram.end());

            for (int i = 0; i < width; i++)
            {
                // std::cout << "Histogram " << i << ": " << histogram[i] << std::endl;
                int bar_height = static_cast<int>((histogram[i] / static_cast<float>(max_value)) * hist_image.rows); // her sütunun yüksekliğini buluyoruz
                cv::line(                                                                                            // başlangıç ve bitiş noktası verilen yerde dik bir çizgi çiz
                    hist_image,
                    cv::Point(i, hist_image.rows),
                    cv::Point(i, hist_image.rows - bar_height),
                    cv::Scalar(255, 255, 255));
            }

            //cv::imshow("Histogram Peaks", hist_image);
            //cv::waitKey(1);
        }

        return histogram;
    }

    std::pair<int, int> histogram_peak()
    {
        // Histogramın ortası
        int midpoint = static_cast<int>(histogram.size() / 2);

        // Sol zirve noktasını bul
        int leftx_base = std::distance(
            histogram.begin(),
            std::max_element(histogram.begin(), histogram.begin() + midpoint));

        // Sağ zirve noktasını bul
        int rightx_base = std::distance(
            histogram.begin(),
            std::max_element(histogram.begin() + midpoint, histogram.end()));

        if (leftx_base != 0 & rightx_base != 0)
        {
            last_peaks.first = leftx_base;
            last_peaks.second = rightx_base;
        }

        //std::cout << "rightx_base: " << rightx_base << std::endl;
        //std::cout << "leftx_base: " << leftx_base << std::endl;

        // Sol ve sağ zirve x koordinatlarını döndür
        return std::make_pair(leftx_base, rightx_base);
    }

    cv::Vec3d fit_poly(const std::vector<double> &x, const std::vector<double> &y)
    {
        int n = x.size();
        // y = ax2 + bx + c diye bir denklemimiz var 
        cv::Mat A(n, 3, CV_64F); // bu denklemin sol tarafını ifade ediyor (x)
        cv::Mat Y(n, 1, CV_64F); // bu denklemin sağ tarafını ifade ediyor (y)

        for (int i = 0; i < n; ++i)
        {
            A.at<double>(i, 0) = x[i] * x[i];
            A.at<double>(i, 1) = x[i];
            A.at<double>(i, 2) = 1;
            Y.at<double>(i, 0) = y[i];
        }

        cv::Mat coeffs;
        cv::solve(A, Y, coeffs, cv::DECOMP_SVD); //verilen x ve y değerlerine göre 2. dereceden denklemin katsayılarını bulur.
        return cv::Vec3d(coeffs.at<double>(0), coeffs.at<double>(1), coeffs.at<double>(2));
    }

    std::pair<cv::Vec3d, cv::Vec3d> get_lane_line_indices_sliding_windows(bool plot = false)
    {
        cv::Mat frame_sliding_window = warped_frame.clone();

        int window_height = warped_frame.rows / no_of_windows; // divided screen heights

        cv::Mat nonzeroMat = warped_frame > 100;
        //std::cout << "nonzeroMat size: " << cv::countNonZero(nonzeroMat) << std::endl;
        //std::cout << "warped_frame size: " << cv::countNonZero(warped_frame) << std::endl;

        // warped frame içinde siyah (0) ve beyaz (255) değerinde
        // pikseller var. Biz de beyaz olanları buluyoruz
        std::vector<cv::Point> nonzeroPoints;
        cv::findNonZero(nonzeroMat, nonzeroPoints);

        std::vector<int> nonzerox, nonzeroy;
        for (const auto &p : nonzeroPoints)
        {
            nonzeroy.push_back(p.y);
            nonzerox.push_back(p.x);
            // beyaz noktaların x ve y değerlerini ayrı ayrı dizilere ekliyoruz
        }

        std::pair<int, int> histogram_peak_base;
        if (histogram_peak().first == 0 || histogram_peak().second == 0)
        {
            histogram_peak_base = last_peaks;
        }
        else
        {
            histogram_peak_base = histogram_peak();
        }

        // auto [leftx_base, rightx_base] = histogram_peak();
        int leftx_current = histogram_peak_base.first;
        int rightx_current = histogram_peak_base.second;

        std::vector<int> left_lane_inds, right_lane_inds;

        // ---- Sliding Windowları Şeritleri Kapsayacak Şekilde Sağ ve Sola Çiziyoruz ----
        for (int window = 0; window < no_of_windows; ++window)
        {
            int win_y_low = warped_frame.rows - (window + 1) * window_height;
            int win_y_high = warped_frame.rows - window * window_height;
            int win_xleft_low = leftx_current - margin;
            int win_xleft_high = leftx_current + margin;
            int win_xright_low = rightx_current - margin;
            int win_xright_high = rightx_current + margin;

            // std::cout << "margin: " << margin << std::endl;
            //std::cout << "rightx_current: " << rightx_current << std::endl;
            //std::cout << "leftx_current: " << leftx_current << std::endl;
            /*
            std::cout << "win_xright_low: " << win_xright_low << std::endl;
            std::cout << "win_xright_high: " << win_xright_high << std::endl;
            std::cout << "win_xleft_low: " << win_xleft_low << std::endl;
            std::cout << "win_xleft_high: " << win_xleft_high << std::endl;
            */

            cv::rectangle(frame_sliding_window,
                          cv::Point(win_xleft_low, win_y_low),
                          cv::Point(win_xleft_high, win_y_high),
                          cv::Scalar(255, 255, 255), 2);

            cv::rectangle(frame_sliding_window,
                          cv::Point(win_xright_low, win_y_low),
                          cv::Point(win_xright_high, win_y_high),
                          cv::Scalar(255, 255, 255), 2);

            std::vector<int> good_left_inds, good_right_inds;
            for (size_t i = 0; i < nonzeroy.size(); ++i)
            {
                if (nonzeroy[i] >= win_y_low && nonzeroy[i] < win_y_high)
                {
                    if (nonzerox[i] >= win_xleft_low && nonzerox[i] < win_xleft_high)
                    {
                        good_left_inds.push_back(i);
                    }
                    if (nonzerox[i] >= win_xright_low && nonzerox[i] < win_xright_high)
                    {
                        good_right_inds.push_back(i);
                    }
                }
            }
            // good_right_inds ve good_left_inds dizileri noktaların kendisini tutmaz sadece noktaların indeks değerlerini tutar
            // nonzerox, nonzeroy gibi dizilere o indeks değerini girerek istenen noktaya ulaşılabilir

            left_lane_inds.insert(left_lane_inds.end(), good_left_inds.begin(), good_left_inds.end());
            right_lane_inds.insert(right_lane_inds.end(), good_right_inds.begin(), good_right_inds.end());
            // insert() ile left_lane_inds ve right_lane_inds vektörlerine good_left_inds ve good_right_inds vektörlerini ekliyoruz

            //std::cout << "good_left_inds: " << good_left_inds.size() << std::endl;
            //std::cout << "minpix: " << minpix << std::endl;

            if (good_left_inds.size() > minpix)
            {
                std::cout << "AAAAAAAAAAAAAAAAAAA" << std::endl;
                int sum = 0;
                for (int idx : good_left_inds)
                    sum += nonzerox[idx];
                leftx_current = sum / good_left_inds.size();
            }
            if (good_right_inds.size() > minpix)
            {
                int sum = 0;
                for (int idx : good_right_inds)
                    sum += nonzerox[idx];
                rightx_current = sum / good_right_inds.size();
            }
        }
        // pencere içerisindeki good indeksler (hem sağ hem de sol için ayrı ayrı)
        // minpix değerinden fazlaysa, o indekslerden noktaların x koordinatlarına ulaşılır
        // ve bu koordinatların ortalaması bulunarak çizginin sayfanın hangi sütununda olduğu belirlenir
        // böylelikle bir sonraki pencere bu sütuna hizalanmış olur.

        std::vector<double> leftx, lefty, rightx, righty;
        for (int idx : left_lane_inds)
        {
            leftx.push_back(nonzerox[idx]);
            lefty.push_back(nonzeroy[idx]);
        }
        for (int idx : right_lane_inds)
        {
            rightx.push_back(nonzerox[idx]);
            righty.push_back(nonzeroy[idx]);
        }

        left_fit = fit_poly(lefty, leftx);
        right_fit = fit_poly(righty, rightx);

        // Step 1: Calculate the x-values for the left and right lane lines
        std::vector<double> ploty, left_fitx, right_fitx;
        for (int i = 0; i < warped_frame.rows; ++i)
        {
            ploty.push_back(i);
            double lx = left_fit[0] * i * i + left_fit[1] * i + left_fit[2];
            double rx = right_fit[0] * i * i + right_fit[1] * i + right_fit[2];
            left_fitx.push_back(lx);
            right_fitx.push_back(rx);
        }

        // Step 2: Create color output image
        cv::Mat color_output;
        cv::cvtColor(warped_frame, color_output, cv::COLOR_GRAY2BGR); // Convert grayscale to color

        // Step 3: Add lane line pixels to the image (red for left, blue for right)
        for (int i : left_lane_inds)
        {
            color_output.at<cv::Vec3b>(nonzeroy[i], nonzerox[i]) = {0, 0, 255}; // Red color for left lane
        }
        for (int i : right_lane_inds)
        {
            color_output.at<cv::Vec3b>(nonzeroy[i], nonzerox[i]) = {255, 0, 0}; // Blue color for right lane
        }

        // Step 5: Plot the detected lane lines on the color output image
        for (size_t i = 0; i < ploty.size(); ++i)
        {
            int y = static_cast<int>(ploty[i]);
            int left_x = static_cast<int>(left_fitx[i]);
            int right_x = static_cast<int>(right_fitx[i]);
            cv::line(color_output, cv::Point(left_x, y), cv::Point(left_x, y), cv::Scalar(255, 255, 0), 2);   // Yellow for left line
            cv::line(color_output, cv::Point(right_x, y), cv::Point(right_x, y), cv::Scalar(255, 255, 0), 2); // Yellow for right line
        }

        // Step 6: Display the final image with the lane lines
        cv::imshow("Detected Lane Lines with Sliding Windows", color_output); // Display the image with lane lines

        // Wait for the user to press a key before closing
        // Wait for key press before closing windows


        cv::imshow("frame_sliding_window", frame_sliding_window);
        cv::waitKey(1);

        return {left_fit, right_fit};

    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    cv::Mat lane_line_markings;
    cv::Mat transformation_matrix;
    cv::Mat inv_transformation_matrix;
    cv::Mat warped_frame;

    cv::Size orig_image_size;

    int width;
    int height;
    int padding;
    std::vector<cv::Point2f> roi_points;
    std::vector<cv::Point2f> desired_roi_points;

    std::vector<int> histogram;

    int no_of_windows = 10; // Number of windows for lane line detection
    int margin;             // Width of each window
    int minpix;             // Min number of pixels to recenter window

    std::pair<int, int> last_peaks;

    cv::Vec3d left_fit;
    cv::Vec3d right_fit;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
