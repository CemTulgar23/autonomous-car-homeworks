#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <algorithm>
#include "edge_detection.h"
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"

class LaneNode : public rclcpp::Node
{
public:
    LaneNode() : Node("lane")
    {

        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/carla/ego_vehicle/rgb_front/image",
            10,
            std::bind(&LaneNode::image_callback, this, std::placeholders::_1));

        ego_vehicle_status_sub = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>(
            "/carla/ego_vehicle/vehicle_status",
            10,
            std::bind(&LaneNode::ege_vehicle_status_callback, this, std::placeholders::_1));

        ego_vehicle_pub = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
            "/carla/ego_vehicle/vehicle_control_cmd", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&LaneNode::control_ego_vehicle, this));
    }

    // private:
    //    const sensor_msgs::msg::Image::SharedPtr msg
    //    cv::Mat image
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // ROS 2 Image mesajını OpenCV formatına çevir
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        orig_frame = image;

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
            cv::Point2f(width / 2 - 175, height - 100), // Top-left corner
            cv::Point2f(width / 2 - 260, height - 1),   // Bottom-left corner
            cv::Point2f(width / 2 + 260, height - 1),   // Bottom-right corner
            cv::Point2f(width / 2 + 175, height - 100)  // Top-right corner
        };

        /*
        roi_points = {
            cv::Point2f(274, 184),  // Top-left corner
            cv::Point2f(0, 337),    // Bottom-left corner
            cv::Point2f(575, 337),  // Bottom-right corner
            cv::Point2f(371, 184)   // Top-right corner
        };
        */

        orig_image_size = image.size();

        lane_line_markings = get_line_markings(image);

        perspective_transform(image, false);

        calculateHistogram(warped_frame, true);

        find_histogram_lane();

        std::pair<int, int> peaks = histogram_peak();
        // std::cout << "Left peak: " << peaks.first << ", Right peak: " << peaks.second << std::endl;

        margin = static_cast<int>((1.0 / 12.0) * width); // Window width is +/- margin
        minpix = static_cast<int>((1.0 / 24.0) * width); // Min no. of pixels to recenter window

        get_lane_line_indices_sliding_windows(true);

        calculate_car_position(true);

        plot_roi(image, roi_points, false);

        // cv::imshow("Image", image);
        // cv::imshow("Lane Line Markings", lane_line_markings);
        // std::cout << "BBBBBBBBBBBBBBBBBBBBBBBBB" << std::endl;
        cv::waitKey(1);
    }

    double calculate_car_position(bool print_to_terminal = false)
    {
        // Arabanın görüntüdeki konumu (kamera merkezde varsayılıyor)
        // double car_location = orig_frame.cols / 2.0;
        double car_location = roi_points[1].x + ((roi_points[2].x - roi_points[1].x) / 2);

        // Sol ve sağ şerit çizgilerinin tabandaki x koordinatları
        double bottom_left = left_fit[0] * height * height + left_fit[1] * height + left_fit[2];
        double bottom_right = right_fit[0] * height * height + right_fit[1] * height + right_fit[2];

        // Şerit merkezinin konumu
        double center_lane = (bottom_right - bottom_left) / 2.0 + bottom_left;

        double XM_PER_PIX = 3.7 / 781.0; // piksel başına santimetre cinsinden değer
        // Araç merkezinin şerit merkezine olan uzaklığı (cm cinsinden)
        double center_offset = (car_location - center_lane) * XM_PER_PIX * 100.0;
        // + -> merkezin sağında
        // - -> merkezin solunda

        if (print_to_terminal)
        {
            std::cout << center_offset << " cm" << std::endl;
        }

        this->center_offset = center_offset;

        return center_offset;
    }

    float ege_vehicle_status_callback(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg)
    {
        ego_vehicle_velocity = msg->velocity;
        return ego_vehicle_velocity;
    }

    void control_ego_vehicle()
    {
        auto msg = carla_msgs::msg::CarlaEgoVehicleControl();

        if (center_offset > 30 || center_offset < -30)
        {

            if (ego_vehicle_velocity > 5)
            {
                msg.throttle = 0.0;
                msg.brake = 0.5;
            }
            else
            {
                msg.throttle = 0.1;
                msg.brake = 0.0;
            }

            if (center_offset < 100 && center_offset > -100)
            {
                msg.steer = -center_offset / 100 * 0.8;
            }
            else
            {
                if (center_offset < 0)
                {
                    msg.steer = 1.0;
                }
                else
                {
                    msg.steer = -1.0;
                }
            }
        }
        else
        {
            if (center_offset > 10.0) // merkezin sağında
            {
                std::cout << "Sağda" << std::endl;
                msg.steer = -0.1;

                if (ego_vehicle_velocity < 10)
                {
                    msg.throttle = 0.1;
                }
                else
                {
                    msg.throttle = 0.0;
                }
            }
            else if (center_offset < -10.0) // merkezin solunda
            {
                std::cout << "Solda" << std::endl;
                msg.steer = 0.1;

                if (ego_vehicle_velocity < 10)
                {
                    msg.throttle = 0.1;
                }
                else
                {
                    msg.throttle = 0.0;
                }
            }
            else
            {
                msg.steer = 0.0;
                msg.throttle = 0.2;
            }
        }

        ego_vehicle_pub->publish(msg);
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

        /*
        for (int y = 0; y < lane_line_markings.rows; ++y)
        {
            for (int x = 0; x < lane_line_markings.cols; ++x)
            {
                uchar pixel = lane_line_markings.at<uchar>(y, x);
                cv::Point point = cv::Point(x, y);
                std::cout << "Pixel(" << y << ", " << x << ") = " << (int)pixel << std::endl;
            }
        }
        */

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
        // cv::waitKey(0); // herhangi bir tuşa kadar bekle
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
            // cv::waitKey(0);
        }
        return warped_frame;
    }

    std::vector<int> calculateHistogram(cv::Mat frame = cv::Mat(), bool plot = true)
    {
        if (frame.empty())
        {
            frame = warped_frame.clone(); // Daha önceki frame kullanılır
        }

        // cv::Point2f left_point = desired_roi_points[0];
        // cv::Point2f right_point = desired_roi_points[3];

        cv::Point2f left_point = cv::Point2f(0, 0);
        cv::Point2f right_point = cv::Point2f(width, 0);

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

            cv::imshow("Histogram Peaks", hist_image);
            // cv::waitKey(0);
        }

        return histogram;
    }

    void find_histogram_lane()
    {
        bool white_center = false;

        int center = width / 2;
        for (size_t i = center - margin; i <= center + margin; i++)
        {
            if (histogram[i] != 0)
            {
                white_center = true;
                break;
            }
        }

        if (white_center == true)
        {
            int left_side_white_number = 0;
            int right_side_white_number = 0;
            for (size_t i = 0; i < center; i++)
            {
                if (histogram[i] != 0)
                {
                    left_side_white_number++;
                }
            }
            for (size_t i = center; i < width; i++)
            {
                if (histogram[i] != 0)
                {
                    right_side_white_number++;
                }
            }
            left_side_white_number > right_side_white_number ? steering_direction = "right" : steering_direction = "left";
        }
        else
        {
            steering_direction = "forward";
        }
    }

    std::pair<int, int> histogram_peak()
    {
        // Histogramın ortası
        int midpoint = static_cast<int>(histogram.size() / 2);

        // Sol zirve noktasını bul
        int leftx_base = std::distance(
            histogram.begin(),
            std::max_element(histogram.begin(), histogram.begin() + midpoint - padding / 2));

        // Sağ zirve noktasını bul
        int rightx_base = std::distance(
            histogram.begin(),
            std::max_element(histogram.begin() + midpoint + padding / 2, histogram.end()));

        if (leftx_base != 0)
        {
            last_peaks.first = leftx_base;
        }
        if (rightx_base != 0)
        {
            last_peaks.second = rightx_base;
        }

        // std::cout << "rightx_base: " << rightx_base << std::endl;
        // std::cout << "leftx_base: " << leftx_base << std::endl;

        // Sol ve sağ zirve x koordinatlarını döndür
        return std::make_pair(leftx_base, rightx_base);
    }

    cv::Vec3d fit_poly(const std::vector<double> &x, const std::vector<double> &y)
    {
        int n = x.size();
        cv::Mat A(n, 3, CV_64F);
        cv::Mat Y(n, 1, CV_64F);

        for (int i = 0; i < n; ++i)
        {
            A.at<double>(i, 0) = x[i] * x[i];
            A.at<double>(i, 1) = x[i];
            A.at<double>(i, 2) = 1;
            Y.at<double>(i, 0) = y[i];
        }

        cv::Mat coeffs;
        cv::solve(A, Y, coeffs, cv::DECOMP_SVD);
        return cv::Vec3d(coeffs.at<double>(0), coeffs.at<double>(1), coeffs.at<double>(2));
    }

    std::pair<cv::Vec3d, cv::Vec3d> get_lane_line_indices_sliding_windows(bool plot = false)
    {
        cv::Mat frame_sliding_window = warped_frame.clone();
        cv::Mat advanced_warped_frame = warped_frame.clone();

        // std::cout << "Type: " << warped_frame.type() << std::endl;

        // 3x3’lük bir kernel (8 komşuluk)
        // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(13, 13));

        // Genişletme işlemi
        // cv::Mat frame_sliding_window;
        // cv::dilate(warped_frame, frame_sliding_window, kernel);

        int window_height = warped_frame.rows / no_of_windows; // divided screen heights

        // cv::Mat nonzeroMat = warped_frame > 100;

        std::vector<int> point_result_list;
        std::vector<cv::Point> point_list;
        int point_resutl_list_one_count = 0;

        for (int y = 0; y < warped_frame.rows; ++y)
        {
            for (int x = 0; x < warped_frame.cols; ++x)
            {

                uchar pixel = warped_frame.at<uchar>(y, x);
                cv::Point point = cv::Point(x, y);
                point_list.push_back(point);

                if (pixel > 1)
                {
                    if (x > 4 && x < width - 5 && y > 4 && y < height - 5)
                    {
                        for (int dy = -5; dy <= 5; ++dy)
                        {
                            for (int dx = -5; dx <= 5; ++dx)
                            {
                                frame_sliding_window.at<uchar>(y + dy, x + dx) = 255;
                                advanced_warped_frame.at<uchar>(y + dy, x + dx) = 255;
                            }
                        }
                    }

                    point_resutl_list_one_count += 1;
                    point_result_list.push_back(1); // şerit
                }
                else
                {
                    point_result_list.push_back(0); // şerit değil
                }
                uchar pixel_value = frame_sliding_window.at<uchar>(y, x);
                // std::cout << "Pixel(" << y << ", " << x << ") = " << (int)pixel_value << std::endl;
            }
        }

        // std::cout << "point_resutl_list_one_count: " << point_resutl_list_one_count << std::endl;

        /*
        std::cout << "nonzeroMat size: " << cv::countNonZero(nonzeroMat) << std::endl;
        std::cout << "warped_frame size: " << cv::countNonZero(warped_frame) << std::endl;
        */
        // warped frame içinde siyah (0) ve beyaz (255) değerinde
        // pikseller var. Biz de beyaz olanları buluyoruz

        /*
        std::vector<cv::Point> nonzeroPoints;
        cv::findNonZero(nonzeroMat, nonzeroPoints);

        std::vector<int> nonzerox, nonzeroy;
        for (const auto &p : nonzeroPoints)
        {
            nonzeroy.push_back(p.y);
            nonzerox.push_back(p.x);
            // beyaz noktaların x ve y değerlerini ayrı ayrı dizilere ekliyoruz
        }
        */

        std::vector<double> left_line_x_list;
        std::vector<double> left_line_y_list;
        std::vector<double> right_line_x_list;
        std::vector<double> right_line_y_list;

        cv::Point recent_left_first_white_point = cv::Point(0, 0);
        cv::Point recent_right_first_white_point = cv::Point(0, 0);
        if (steering_direction == "forward")
        {
            recent_left_first_white_point = cv::Point(width / 4, 0);
            recent_right_first_white_point = cv::Point(width * 3 / 4, height);
        }
        else if (steering_direction == "left")
        {
            recent_left_first_white_point = cv::Point(0, 0);
            recent_right_first_white_point = cv::Point(width * 3 / 4 + margin, height);
        }
        else if (steering_direction == "right")
        {
            recent_left_first_white_point = cv::Point(width / 4 - margin, 0);
            recent_right_first_white_point = cv::Point(width - margin * 2, height);
        }

        for (int y = height - 1; y >= 0; y -= window_height)
        {
            cv::Point left_first_white_point = cv::Point(0, 0);
            cv::Point right_first_white_point = cv::Point(0, 0);

            if (steering_direction == "forward")
            {
                for (int x = width / 2; x < width; x++)
                {
                    if (advanced_warped_frame.at<uchar>(y, x) > 1)
                    {
                        right_first_white_point = cv::Point(x, y);
                        recent_right_first_white_point = right_first_white_point;
                        break;
                    }
                }
                if (right_first_white_point.x == 0 && right_first_white_point.y == 0)
                {
                    right_first_white_point = recent_right_first_white_point;
                }

                for (int x = width / 2; x > 0; x--)
                {
                    if (advanced_warped_frame.at<uchar>(y, x) > 1)
                    {
                        left_first_white_point = cv::Point(x, y);
                        recent_left_first_white_point = left_first_white_point;
                        break;
                    }
                }
                if (left_first_white_point.x == 0 && left_first_white_point.y == 0)
                {
                    left_first_white_point = recent_left_first_white_point;
                }
            }
            else if (steering_direction == "left")
            {
                for (int x = width / 2 - margin * 2; x < width; x++)
                {
                    if (advanced_warped_frame.at<uchar>(y, x) > 1)
                    {
                        right_first_white_point = cv::Point(x, y);
                        recent_right_first_white_point = right_first_white_point;
                        break;
                    }
                }
                if (right_first_white_point.x == 0 && right_first_white_point.y == 0)
                {
                    right_first_white_point = recent_right_first_white_point;
                }

                for (int x = right_first_white_point.x - margin * 2; x > 0; x--)
                {
                    if (advanced_warped_frame.at<uchar>(y, x) > 1)
                    {
                        left_first_white_point = cv::Point(x, y);
                        recent_left_first_white_point = left_first_white_point;
                        break;
                    }
                }
                if (left_first_white_point.x == 0 && left_first_white_point.y == 0)
                {
                    left_first_white_point = recent_left_first_white_point;
                }
            }
            else if (steering_direction == "right")
            {
                for (int x = width / 2 + margin * 2; x > 0; x--)
                {
                    if (advanced_warped_frame.at<uchar>(y, x) > 1)
                    {
                        left_first_white_point = cv::Point(x, y);
                        recent_left_first_white_point = left_first_white_point;
                        break;
                    }
                }
                if (left_first_white_point.x == 0 && left_first_white_point.y == 0)
                {
                    left_first_white_point = recent_left_first_white_point;
                }

                for (int x = left_first_white_point.x + margin * 2; x < width; x++)
                {
                    if (advanced_warped_frame.at<uchar>(y, x) > 1)
                    {
                        right_first_white_point = cv::Point(x, y);
                        recent_right_first_white_point = right_first_white_point;
                        break;
                    }
                }
                if (right_first_white_point.x == 0 && right_first_white_point.y == 0)
                {
                    right_first_white_point = recent_right_first_white_point;
                }
            }

            // std::cout << "left_first_white_point" << left_first_white_point << std::endl;
            // std::cout << "right_first_white_point" << right_first_white_point << std::endl;
            // std::cout << "steering_direction" << steering_direction << std::endl;
            uchar pixel = advanced_warped_frame.at<uchar>(0, 0);
            // std::cout << "Pixel: " << (int)pixel << std::endl;

            int win_low_y = y;
            int win_high_y = y - window_height;
            int win_left_low_x = left_first_white_point.x + margin * 2;
            int win_left_high_x = left_first_white_point.x;
            int win_right_low_x = right_first_white_point.x;
            int win_right_high_x = right_first_white_point.x - margin * 2;

            int left_line_x = win_left_low_x + (win_left_high_x - win_left_low_x) / 2;
            int left_line_y = win_low_y + (win_high_y - win_low_y) / 2;
            int right_line_x = win_right_low_x + (win_right_high_x - win_right_low_x) / 2;
            int right_line_y = win_low_y + (win_high_y - win_low_y) / 2;

            left_line_x_list.push_back(left_line_x);
            left_line_y_list.push_back(left_line_y);
            right_line_x_list.push_back(right_line_x);
            right_line_y_list.push_back(right_line_y);

            cv::rectangle(frame_sliding_window,
                          cv::Point(win_left_low_x, win_low_y),
                          cv::Point(win_left_high_x, win_high_y),
                          cv::Scalar(80, 80, 80), 2);

            cv::rectangle(frame_sliding_window,
                          cv::Point(win_right_low_x, win_low_y),
                          cv::Point(win_right_high_x, win_high_y),
                          cv::Scalar(160, 160, 160), 2);
        }
        cv::imshow("frame_sliding_window", frame_sliding_window);

        try
        {
            left_fit = fit_poly(left_line_y_list, left_line_x_list);
            right_fit = fit_poly(right_line_y_list, right_line_x_list);
        }
        catch (const std::exception &e)
        {
            std::cout << "ERROR ERROR ERROR ERROR" << std::endl;
            std::cerr << e.what() << '\n';
        }

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

        for (int y : left_line_y_list)
        {
            for (int x : left_line_x_list)
            {
                color_output.at<cv::Vec3b>(y, x) = {0, 0, 255}; // Red color for left lane
            }
        }

        for (int y : right_line_y_list)
        {
            for (int x : right_line_x_list)
            {
                color_output.at<cv::Vec3b>(y, x) = {255, 0, 0}; // Blue color for right lane
            }
        }

        /*
        for (int i : left_lane_inds)
        {
            color_output.at<cv::Vec3b>(point_list[i].y, point_list[i].x) = {0, 0, 255}; // Red color for left lane
        }
        for (int i : right_lane_inds)
        {
            color_output.at<cv::Vec3b>(point_list[i].y, point_list[i].x) = {255, 0, 0}; // Blue color for right lane
        }
        */

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

        // cv::imshow("frame_sliding_window", frame_sliding_window);
        //  Wait for the user to press a key before closing
        //  cv::waitKey(0);

        /*
        // Siyah bir görüntü oluştur
        cv::Mat image = cv::Mat::zeros(height, width, CV_8UC1); // Tek kanal (grayscale)

        for (int idx = 0; idx < point_result_list.size(); idx++)
        {
            if (point_result_list[idx] == 1)
            {
                // std::cout << "XXXXXXXXXXXXXXXXXXXXXX" << std::endl;
                int row = idx / width;
                int col = idx % width;

                if (row >= 0 && row < height && col >= 0 && col < width)
                {
                    image.at<uchar>(row, col) = 255; // Beyaz yap
                }
            }
        }

        // Sonuç görüntüsünü göster
        cv::imshow("Result", image);

        return {left_fit, right_fit};
        */
    }

    /*
    cv::Mat fitPolynomial(const std::vector<int> &x, const std::vector<int> &y)
    {
        cv::Mat A(x.size(), 3, CV_64F); // Matrix for quadratic terms (x^2, x, 1)
        cv::Mat B(y);                   // Output vector for y values

        // Fill matrix A with x^2, x, and 1
        for (size_t i = 0; i < x.size(); ++i)
        {
            A.at<double>(i, 0) = x[i] * x[i]; // x^2
            A.at<double>(i, 1) = x[i];        // x
            A.at<double>(i, 2) = 1;           // constant term (1)
        }

        // Solve for polynomial coefficients (a, b, c)
        cv::Mat coefficients;
        cv::solve(A, B, coefficients, cv::DECOMP_SVD); // Use least squares solution
        return coefficients;

        // Return the polynomial coefficients(katsayı) a, b, c
        //return cv::Vec3d(coefficients.at<double>(0), coefficients.at<double>(1), coefficients.at<double>(2));
    }

    void getLaneLineIndicesSlidingWindows(bool plot = false)
    {
        cv::Mat frame_sliding_window = warped_frame.clone();

        // Kayma pencerelerinin yüksekliği
        int windowHeight = static_cast<int>(warped_frame.rows / no_of_windows);

        // Beyaz piksellerin koordinatlarını al
        cv::Mat nonzeroMat = warped_frame > 0; // Beyaz pikselleri tespit et
        std::vector<cv::Point> nonzeroPoints;
        cv::findNonZero(nonzeroMat, nonzeroPoints); // beyaz piksellerin koordinatları artık nonzeroPoints içerisinde

        std::vector<int> nonzerox, nonzeroy;
        for (const auto &pt : nonzeroPoints)
        {
            nonzerox.push_back(pt.x);
            nonzeroy.push_back(pt.y);
        }

        // Sol ve sağ yol çizgilerinin piksellerini depolamak için
        std::vector<std::vector<int>> leftLaneInds, rightLaneInds;

        // Başlangıç x koordinatlarını histogramdan al
        std::pair<int, int> xBase = histogram_peak();
        int leftxCurrent = xBase.first;
        int rightxCurrent = xBase.second;

        // Pencereleri kaydırarak her biri için işlem yap
        for (int window = 0; window < no_of_windows; ++window)
        {
            // Pencere sınırlarını belirle
            int winYLow = warped_frame.rows - (window + 1) * windowHeight;
            int winYHigh = warped_frame.rows - window * windowHeight;
            int winXLeftLow = leftxCurrent - margin;
            int winXLeftHigh = leftxCurrent + margin;
            int winXRightLow = rightxCurrent - margin;
            int winXRightHigh = rightxCurrent + margin;

            cv::rectangle(frame_sliding_window, cv::Point(winXLeftLow, winYLow),
                          cv::Point(winXLeftHigh, winYHigh), cv::Scalar(255, 255, 255), 2);

            cv::rectangle(frame_sliding_window, cv::Point(winXRightLow, winYLow),
                          cv::Point(winXRightHigh, winYHigh), cv::Scalar(255, 255, 255), 2);

            // Sol ve sağ yol çizgileri için pikselleri bul
            std::vector<int> goodLeftInds, goodRightInds;
            for (size_t i = 0; i < nonzeroy.size(); ++i)
            {
                if (nonzeroy[i] >= winYLow && nonzeroy[i] < winYHigh)
                {
                    if (nonzerox[i] >= winXLeftLow && nonzerox[i] < winXLeftHigh)
                        goodLeftInds.push_back(i);
                    if (nonzerox[i] >= winXRightLow && nonzerox[i] < winXRightHigh)
                        goodRightInds.push_back(i);
                }
            }

            // Pikselleri listeye ekle
            leftLaneInds.push_back(goodLeftInds);
            rightLaneInds.push_back(goodRightInds);

            // Yeterli piksel bulunduysa, pencereyi yeniden merkezle
            if (goodLeftInds.size() > minpix)
            {
                leftxCurrent = static_cast<int>(std::accumulate(goodLeftInds.begin(), goodLeftInds.end(), 0.0) / goodLeftInds.size());
            }
            if (goodRightInds.size() > minpix)
            {
                rightxCurrent = static_cast<int>(std::accumulate(goodRightInds.begin(), goodRightInds.end(), 0.0) / goodRightInds.size());
            }
        }

        // İndeksleri birleştir
        std::vector<int> leftLaneFinalInds, rightLaneFinalInds;
        for (const auto &inds : leftLaneInds)
            leftLaneFinalInds.insert(leftLaneFinalInds.end(), inds.begin(), inds.end());
        for (const auto &inds : rightLaneInds)
            rightLaneFinalInds.insert(rightLaneFinalInds.end(), inds.begin(), inds.end());

        // Koordinatları al
        std::vector<int> leftX, leftY, rightX, rightY;
        for (int idx : leftLaneFinalInds)
        {
            leftX.push_back(nonzerox[idx]);
            leftY.push_back(nonzeroy[idx]);
        }
        for (int idx : rightLaneFinalInds)
        {
            rightX.push_back(nonzerox[idx]);
            rightY.push_back(nonzeroy[idx]);
        }

        // 2. dereceden polinom eğrisi uydurma
        cv::Mat leftFit = fitPolynomial(leftY, leftX);
        cv::Mat rightFit = fitPolynomial(rightY, rightX);

        if (plot)
        {
            plotResults(leftX, leftY, rightX, rightY, leftFit, rightFit);
        }

        leftFit_ = leftFit;
        rightFit_ = rightFit;
    }
    */

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr ego_vehicle_status_sub;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr ego_vehicle_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::Mat orig_frame;
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

    std::pair<int, int> last_peaks = {0, 0};

    cv::Vec3d left_fit;
    cv::Vec3d right_fit;

    double center_offset;
    float ego_vehicle_velocity;

    std::string steering_direction;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

/*
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LaneNode>();

    std::string imagePath = "/home/tulgar/catkin_ws/src/lane_detection/include/road4.png";

    cv::Mat image = cv::imread(imagePath);

    node->image_callback(image);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
*/
