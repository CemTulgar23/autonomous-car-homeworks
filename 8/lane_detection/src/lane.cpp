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

    private:
    //   const sensor_msgs::msg::Image::SharedPtr msg
    //   cv::Mat image
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
            cv::Point2f(width / 2 - 215, height - 50), // Top-left corner
            cv::Point2f(width / 2 - 260, height - 1),   // Bottom-left corner
            cv::Point2f(width / 2 + 260, height - 1),   // Bottom-right corner
            cv::Point2f(width / 2 + 215, height - 50)  // Top-right corner
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

        perspective_transform(image, true);

        calculateHistogram(warped_frame, true);

        std::pair<int, int> peaks = histogram_peak();
        // std::cout << "Left peak: " << peaks.first << ", Right peak: " << peaks.second << std::endl;

        margin = static_cast<int>((1.0 / 12.0) * width); // Window width is +/- margin
        minpix = static_cast<int>((1.0 / 24.0) * width); // Min no. of pixels to recenter window

        //get_lane_line_indices_sliding_windows(true);

        // calculate_car_position(true);

        plot_roi(image, roi_points, false);

        cv::imshow("Image", image);
        //cv::imshow("Lane Line Markings", lane_line_markings);
        std::cout << "BBBBBBBBBBBBBBBBBBBBBBBBB" << std::endl;
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

        center_offset = center_offset;
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
        // std::cout << "dsfsssssssssssssssssssssssss" << std::endl;
        auto msg = carla_msgs::msg::CarlaEgoVehicleControl();

        if (center_offset > 3.0) // merkezin sağında
        {
            std::cout << "Sağdaaaaa" << std::endl;
            msg.steer = -0.1;
        }
        else if (center_offset < -3.0) // merkezin solunda
        {
            msg.steer = +0.1;
        }
        else
        {
            msg.steer = 0.0;
        }

        if (ego_vehicle_velocity < 10)
        {
            msg.throttle = 0.2;
        }
        else
        {
            msg.throttle = 0.0;
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

        cv::Point2f left_point = desired_roi_points[0];
        cv::Point2f right_point = desired_roi_points[3];

        // std::cout << "Left x: " << left_point.x << ", Right x: " << right_point.x << std::endl;

        // int histogram_width = right_point.x - left_point.x;

        // Histogramı sıfırla
        histogram = std::vector<int>(width, 0);

        // Alt yarıdan beyaz pikselleri say
        for (int y = 0; y < height / 2; y++)
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

        std::cout << "rightx_base: " << rightx_base << std::endl;
        std::cout << "leftx_base: " << leftx_base << std::endl;

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

                if (pixel > 100)
                {
                    point_resutl_list_one_count += 1;
                    point_result_list.push_back(1); // şerit
                }
                else
                {
                    point_result_list.push_back(0); // şerit değil
                }
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

        if (last_peaks.first == 0)
        {
            last_peaks.first = width / 4;
        }
        if (last_peaks.second == 0)
        {
            last_peaks.second = width * 3 / 4;
        }

        std::pair<int, int> histogram_peak_base;
        if (histogram_peak().first == 0)
        {
            histogram_peak_base.first = last_peaks.first;
        }
        else
        {
            histogram_peak_base.first = histogram_peak().first;
        }

        if (histogram_peak().second == 0)
        {
            histogram_peak_base.second = last_peaks.second;
        }
        else
        {
            histogram_peak_base.second = histogram_peak().second;
        }

        /*
        std::pair<int, int> histogram_peak_base;
        if (histogram_peak().first == 0 || histogram_peak().second == 0)
        {
            histogram_peak_base = last_peaks;
        }
        else
        {
            histogram_peak_base = histogram_peak();
        }
        */

        // auto [leftx_base, rightx_base] = histogram_peak();
        int leftx_current = histogram_peak_base.first;
        int rightx_current = histogram_peak_base.second;

        // std::vector<int> left_lane_inds, right_lane_inds;
        // std::vector<int> proper_left_lane_inds, proper_right_lane_inds;

        std::vector<int> left_lane_inds, right_lane_inds;
        static std::vector<int> next_window_good_left_inds, next_window_good_right_inds;
        static bool cond = true;

        // std::cout << "image size: " << width * height << std::endl;
        if (cond)
        {
            // std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
            for (size_t i = 0; i < window_height; i++)
            {
                // std::cout << "left index: " << width / 4 + (height - i - 1) * width << std::endl;
                for (size_t j = 0; j < 3; j++)
                {
                    next_window_good_left_inds.push_back(width / 4 + j + (height - i - 1) * width);
                }
            }
            for (size_t i = 0; i < window_height; i++)
            {
                // std::cout << "right index: " << (width / 4 * 3) + (height - i - 1) * width << std::endl;
                for (size_t j = 0; j < 3; j++)
                {
                    next_window_good_right_inds.push_back((width / 4 * 3) + j + (height - i - 1) * width);
                }
            }
        }
        else
        {
            int inds_gap = (height - window_height) * width;
            std::transform(
                next_window_good_left_inds.begin(),
                next_window_good_left_inds.end(),
                next_window_good_left_inds.begin(),
                [inds_gap](int val)
                { return val + inds_gap; });

            std::transform(
                next_window_good_right_inds.begin(),
                next_window_good_right_inds.end(),
                next_window_good_right_inds.begin(),
                [inds_gap](int val)
                { return val + inds_gap; });
        }
        cond = false;

        int good_right_inds_sum = 0;
        int good_left_inds_sum = 0;
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
            // std::cout << "rightx_current: " << rightx_current << std::endl;
            std::cout << "leftx_current: " << leftx_current << std::endl;
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
            int good_left_inds_count = 0;
            for (size_t i = 0; i < point_result_list.size(); i++)
            {
                if (point_result_list[i] == 1)
                {
                    cv::Point point = point_list[i];
                    if (point.y >= win_y_low && point.y < win_y_high)
                    {
                        if (point.x >= win_xleft_low && point.x < win_xleft_high)
                        {
                            good_left_inds_count += 1;
                            good_left_inds.push_back(i);
                        }
                        if (point.x >= win_xright_low && point.x < win_xright_high)
                        {
                            good_right_inds.push_back(i);
                        }
                    }
                }
            }
            // std::cout << "good_left_inds_count: " << good_left_inds_count << std::endl;

            /*
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
            */
            // good_right_inds ve good_left_inds dizileri noktaların kendisini tutmaz sadece noktaların indeks değerlerini tutar
            // nonzerox, nonzeroy gibi dizilere o indeks değerini girerek istenen noktaya ulaşılabilir

            // insert() ile left_lane_inds ve right_lane_inds vektörlerine good_left_inds ve good_right_inds vektörlerini ekliyoruz

            // int inds_gap =(win_xleft_high - win_xleft_low);
            int inds_gap = width * window_height;
            if (good_left_inds.size() > minpix)
            {
                // std::cout << "Entered good_left_inds if" << std::endl;
                next_window_good_left_inds.clear();
                for (size_t i = 0; i < good_left_inds.size(); i++)
                {
                    next_window_good_left_inds.push_back(good_left_inds[i] - inds_gap);
                }

                int sum = 0;
                for (int idx : good_left_inds)
                    sum += point_list[idx].x;
                leftx_current = sum / good_left_inds.size();
            }
            else
            {
                std::cout << "Entered good_left_inds else" << std::endl;
                for (size_t i = 0; i < next_window_good_left_inds.size(); i++)
                {
                    good_left_inds.push_back(next_window_good_left_inds[i]);
                }
                std::transform(
                    next_window_good_left_inds.begin(),
                    next_window_good_left_inds.end(),
                    next_window_good_left_inds.begin(),
                    [inds_gap](int val)
                    { return val - inds_gap; });

                int sum = 0;
                for (int idx : good_left_inds)
                    point_result_list[idx] = 1;
            }

            if (good_right_inds.size() > minpix)
            {
                // std::cout << "Entered good_right_inds if" << std::endl;
                next_window_good_right_inds.clear();
                for (size_t i = 0; i < good_right_inds.size(); i++)
                {
                    next_window_good_right_inds.push_back(good_right_inds[i] - inds_gap);
                }
                int sum = 0;
                for (int idx : good_right_inds)
                    sum += point_list[idx].x;
                rightx_current = sum / good_right_inds.size();
            }
            else
            {
                // std::cout << "Entered good_right_inds else" << std::endl;
                for (size_t i = 0; i < next_window_good_right_inds.size(); i++)
                {
                    good_right_inds.push_back(next_window_good_right_inds[i]);
                }
                std::transform(
                    next_window_good_right_inds.begin(),
                    next_window_good_right_inds.end(),
                    next_window_good_right_inds.begin(),
                    [inds_gap](int val)
                    { return val - inds_gap; });
                int sum = 0;
                for (int idx : good_right_inds)
                    point_result_list[idx] = 1;
            }

            left_lane_inds.insert(left_lane_inds.end(), good_left_inds.begin(), good_left_inds.end());
            right_lane_inds.insert(right_lane_inds.end(), good_right_inds.begin(), good_right_inds.end());

            int count = 0;
            for (size_t i = 0; i < point_result_list.size(); i++)
            {
                if (point_result_list[i] == 1)
                {
                    count += 1;
                }
            }
            good_right_inds_sum += good_right_inds.size();
            good_left_inds_sum += good_left_inds.size();

            /*
            std::cout << "point_result_list size: " << count << std::endl;

            std::cout << "good_left_inds: " << good_left_inds.size() << std::endl;
            std::cout << "next_window_good_left_inds: " << next_window_good_left_inds.size() << std::endl;
            std::cout << "good_left_inds_sum: " << good_left_inds_sum << std::endl;
            std::cout << "good_right_inds: " << good_right_inds.size() << std::endl;
            std::cout << "good_right_inds_sum: " << good_right_inds_sum << std::endl;
            std::cout << "next_window_good_right_inds: " << next_window_good_right_inds.size() << std::endl;
            std::cout << "minpix: " << minpix << std::endl;
            */
        }

        // pencere içerisindeki good indeksler (hem sağ hem de sol için ayrı ayrı)
        // minpix değerinden fazlaysa, o indekslerden noktaların x koordinatlarına ulaşılır
        // ve bu koordinatların ortalaması bulunarak çizginin sayfanın hangi sütununda olduğu belirlenir
        // böylelikle bir sonraki pencere bu sütuna hizalanmış olur.
        /*
        for (int i = 0; i < point_result_list.size(); i++)
        {
            if (point_result_list[i] == 1 && point_list[i].x < width / 2)
            {
                left_lane_inds.push_back(i);
            }
            if (point_result_list[i] == 1 && point_list[i].x > width / 2)
            {
                right_lane_inds.push_back(i);
            }
        }
        */
        // std::cout << "left_lane_inds: " << left_lane_inds.size() << std::endl;
        // std::cout << "right_lane_inds: " << right_lane_inds.size() << std::endl;

        std::vector<double> leftx, lefty, rightx, righty;
        for (int idx : left_lane_inds)
        {
            leftx.push_back(point_list[idx].x);
            lefty.push_back(point_list[idx].y);
        }
        for (int idx : right_lane_inds)
        {
            rightx.push_back(point_list[idx].x);
            righty.push_back(point_list[idx].y);
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
            color_output.at<cv::Vec3b>(point_list[i].y, point_list[i].x) = {0, 0, 255}; // Red color for left lane
        }
        for (int i : right_lane_inds)
        {
            color_output.at<cv::Vec3b>(point_list[i].y, point_list[i].x) = {255, 0, 0}; // Blue color for right lane
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

        cv::imshow("frame_sliding_window", frame_sliding_window);
        // Wait for the user to press a key before closing
        // cv::waitKey(0);

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

    std::string imagePath = "/home/tulgar/catkin_ws/src/lane_detection/include/road.png";

    cv::Mat image = cv::imread(imagePath);

    node->image_callback(image);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
*/
