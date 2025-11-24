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
#include "nav_msgs/msg/odometry.hpp"

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

        ego_vehicle_odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/carla/ego_vehicle/odometry",
            10,
            std::bind(&LaneNode::ego_vehicle_odometry_callback, this, std::placeholders::_1));

        ego_vehicle_pub = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
            "/carla/ego_vehicle/vehicle_control_cmd", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&LaneNode::control_ego_vehicle, this));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // ROS 2 Image mesajını OpenCV formatına çevir
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        orig_frame = image;

        width = image.cols;
        height = image.rows;

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

        orig_image_size = image.size();

        cv::imshow("Image", image);

        lane_line_markings = get_line_markings(image);
        cv::imshow("Lane Line Markings", lane_line_markings);

        perspective_transform(lane_line_markings, true);
        cv::imshow("Warped Frame", warped_frame);

        calculateHistogram(warped_frame, true);

        find_histogram_lane();

        margin = static_cast<int>((1.0 / 12.0) * width); // Window width is +/- margin
        minpix = static_cast<int>((1.0 / 24.0) * width); // Min no. of pixels to recenter window

        std::pair<int, int> peaks = histogram_peak();
        
        get_lane_line_indices_sliding_windows(true);

        calculate_car_position(true);

        plot_roi(image, roi_points, true);

        cv::waitKey(1);
    }

    double calculate_car_position(bool print_to_terminal = false)
    {
        // Arabanın görüntüdeki konumu (kamera merkezde varsayılıyor)
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

    float ege_vehicle_status_callback(const carla_msgs ::msg::CarlaEgoVehicleStatus::SharedPtr msg)
    {
        ego_vehicle_velocity = msg->velocity;
        return ego_vehicle_velocity;
    }

    void ego_vehicle_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        ego_vehicle_current_point = msg->pose.pose.position;
    }

    void control_ego_vehicle()
    {
        auto msg = carla_msgs::msg::CarlaEgoVehicleControl();

        // Eğer hedef bölgeye gelindiyse dur.
        if (isInsidePolygon(polygon, ego_vehicle_current_point))
        {
            msg.steer = 0.0;
            msg.throttle = 0.0;
            msg.brake = 1.0;
        }

        if (center_offset > 30 || center_offset < -30) // viraj ya da keskin dönüş
        {

            if (ego_vehicle_velocity > 5) // hızı 5'ten büyük olmasın
            {
                msg.throttle = 0.0;
                msg.brake = 0.5;
            }
            else
            {
                msg.throttle = 0.1;
                msg.brake = 0.0;
            }

            if (center_offset < 100 && center_offset > -100) // virajlar
            {
                msg.steer = -center_offset / 100 * 0.4;
            }
            else // keskin dönüşler
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
        else // düz yolda şerit ortalama
        {
            if (center_offset > 10.0) // merkezin sağında
            {
                std::cout << "Sağda" << std::endl;
                msg.steer = -0.1;

                if (ego_vehicle_velocity < 10) // hızı 10'dan büyük olmasın
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

                if (ego_vehicle_velocity < 10) // hızı 10'dan büyük olmasın
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
                if (ego_vehicle_velocity < 15) // hızı 15'dan büyük olmasın
                {
                    msg.steer = 0.0;
                    msg.throttle = 0.2;
                }
                else
                {
                    msg.throttle = 0.0;
                }
            }
        }

        ego_vehicle_pub->publish(msg);
    }

    bool isInsidePolygon(const std::vector<std::pair<float, float>> &polygon, geometry_msgs::msg::Point ego_vehicle_current_point)
    {
        float x = ego_vehicle_current_point.x;
        float y = ego_vehicle_current_point.y;

        int i, j, nvert = polygon.size();
        bool inside = false;
        for (i = 0, j = nvert - 1; i < nvert; j = i++)
        {
            float xi = polygon[i].first, yi = polygon[i].second;
            float xj = polygon[j].first, yj = polygon[j].second;

            if (((yi > y) != (yj > y)) &&
                (x < (xj - xi) * (y - yi) / (yj - yi + 1e-9) + xi))
            {
                inside = !inside;
            }
        }
        return inside;
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
        cv::Mat frame = lane_line_markings;

        // 1. Perspektif dönüşüm matrisini hesapla
        transformation_matrix = cv::getPerspectiveTransform(roi_points, desired_roi_points);

        // 2. Ters dönüşüm matrisini hesapla
        inv_transformation_matrix = cv::getPerspectiveTransform(desired_roi_points, roi_points);

        // 3. Perspektif dönüşümü uygula
        cv::warpPerspective(frame, warped_frame, transformation_matrix, orig_image_size, cv::INTER_LINEAR);

        // 4. Görsel olarak çizmek istiyorsan
        if (plot)
        {
            cv::Mat warped_copy = warped_frame.clone();
            std::vector<std::vector<cv::Point>> poly_points(1);
            for (const auto &pt : desired_roi_points)
                poly_points[0].push_back(pt);

            // Çokgen çiz
            cv::polylines(warped_copy, poly_points, true, cv::Scalar(147, 20, 255), 3);

            // Görüntüyü göster
            cv::imshow("Warped Coppy", warped_copy);
        }
        return warped_frame;
    }

    std::vector<int> calculateHistogram(cv::Mat frame = cv::Mat(), bool plot = true)
    {
        if (frame.empty())
        {
            frame = warped_frame.clone(); // Daha önceki frame kullanılır
        }

        cv::Point2f left_point = cv::Point2f(0, 0);
        cv::Point2f right_point = cv::Point2f(width, 0);

        // Histogramı sıfırla
        histogram = std::vector<int>(width, 0);

        // Sütunlardaki beyaz pikselleri say
        for (int y = 0; y < height; y++)
        {
            for (int x = left_point.x; x < right_point.x + 1; x++)
            {
                unsigned char pixel = frame.at<unsigned char>(y, x);
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

        int window_height = warped_frame.rows / no_of_windows; // divided screen heights

        // Warped_frame'de buluanan beyaz noktaları erafındaki pikselleri de beyaz yaparak daha da belirginleştiriyoruz.
        for (int y = 0; y < warped_frame.rows; ++y)
        {
            for (int x = 0; x < warped_frame.cols; ++x)
            {
                uchar pixel = warped_frame.at<uchar>(y, x);
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
                }
            }
        }

        // Kayan pencere oluşturduktan sonra bulduğumuz orta noktaların koordinatlarını atacağımız listeler
        std::vector<double> left_line_x_list;
        std::vector<double> left_line_y_list;
        std::vector<double> right_line_x_list;
        std::vector<double> right_line_y_list;

        cv::Point recent_left_first_white_point = cv::Point(0, 0);
        cv::Point recent_right_first_white_point = cv::Point(0, 0);
        if (steering_direction == "forward")
        {
            recent_left_first_white_point = cv::Point(width / 4, height);
            recent_right_first_white_point = cv::Point(width * 3 / 4, height);
        }
        else if (steering_direction == "left")
        {
            recent_left_first_white_point = cv::Point(0, height);
            recent_right_first_white_point = cv::Point(width * 3 / 4 + margin, height);
        }
        else if (steering_direction == "right")
        {
            recent_left_first_white_point = cv::Point(width / 4 - margin, height);
            recent_right_first_white_point = cv::Point(width, height);
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

            // Bulduğumuz noktalara göre kayan pencerelerin sınırlarını belirliyoruz.
            int win_low_y = y;
            int win_high_y = y - window_height;
            int win_left_low_x = left_first_white_point.x + margin * 2;
            int win_left_high_x = left_first_white_point.x;
            int win_right_low_x = right_first_white_point.x;
            int win_right_high_x = right_first_white_point.x - margin * 2;

            // Kayan pencerelerin orta noktalarının x ve y değerlerini buluyoruz.
            int left_line_x = win_left_low_x + (win_left_high_x - win_left_low_x) / 2;
            int left_line_y = win_low_y + (win_high_y - win_low_y) / 2;
            int right_line_x = win_right_low_x + (win_right_high_x - win_right_low_x) / 2;
            int right_line_y = win_low_y + (win_high_y - win_low_y) / 2;

            // ulduğumuz bu x ve y değerlerini dizilerin içine atıyoruz.
            left_line_x_list.push_back(left_line_x);
            left_line_y_list.push_back(left_line_y);
            right_line_x_list.push_back(right_line_x);
            right_line_y_list.push_back(right_line_y);

            // Sol kayan pencereyi çiziyoruz
            cv::rectangle(frame_sliding_window,
                          cv::Point(win_left_low_x, win_low_y),
                          cv::Point(win_left_high_x, win_high_y),
                          cv::Scalar(80, 80, 80), 2);

            // Sağ kayan pencereyi çiziyoruz
            cv::rectangle(frame_sliding_window,
                          cv::Point(win_right_low_x, win_low_y),
                          cv::Point(win_right_high_x, win_high_y),
                          cv::Scalar(160, 160, 160), 2);
        }
        cv::imshow("Frame Sliding Window", frame_sliding_window);

        try
        {
            // Bulduğumuz kayan pencerelerin orta noktalarını fit_poly() fonskiyonuna atarak 2. dereceden denklem oluşturuyoruz
            // Bu denklem şeridi temsil ediyor.
            left_fit = fit_poly(left_line_y_list, left_line_x_list);
            right_fit = fit_poly(right_line_y_list, right_line_x_list);
        }
        catch (const std::exception &e)
        {
            std::cout << "ERROR, While the system was trying to create a curve from center of the sliding windows." << std::endl;
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
        cv::imshow("Detected Lane Lines With Sliding Windows", color_output); // Display the image with lane lines
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr ego_vehicle_status_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_vehicle_odometry_sub;
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

    std::pair<int, int> last_peaks = {0, 0}; // Histogramın tepe noktaları

    cv::Vec3d left_fit;  // Sol şerit
    cv::Vec3d right_fit; // Sağ şerit

    double center_offset;
    float ego_vehicle_velocity;
    geometry_msgs::msg::Point ego_vehicle_current_point;

    std::string steering_direction;

    std::vector<std::pair<float, float>> polygon = {// Hedef Bölge
                                                    {-62.3f, 44.0f},
                                                    {-63.0f, 41.1f},
                                                    {-68.7f, 35.0f},
                                                    {-70.9f, 34.5f}};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
