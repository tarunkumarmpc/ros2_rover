#ifndef ARROW_GUIDANCE_NODE_HPP_
#define ARROW_GUIDANCE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <image_transport/image_transport.hpp>
#include <mutex>
#include <memory>
#include <iomanip>
#include <sstream>
#include <set>
#include <chrono>
#include <map>
#include <algorithm>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

// Helper: Map string to OpenCV ArUco dictionary enum
static cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_from_string(const std::string& name) {
    static const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dict_map = {
        {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
        {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
        {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
        {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
        {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
        {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
        {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
        {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
        {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
        {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
        {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
        {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
        {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
        {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
        {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
        {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
        {"DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5},
        {"DICT_APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9},
        {"DICT_APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10},
        {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11}
    };
    auto it = dict_map.find(name);
    if (it != dict_map.end())
        return it->second;
    // Default
    return cv::aruco::DICT_4X4_50;
}

class ArrowGuidanceNode : public rclcpp::Node {
public:
    ArrowGuidanceNode();

    void initialize();

private:
    // Parameters
    double aruco_marker_size_;
    std::string arrow_color_;
    std::vector<int64_t> target_marker_ids_;
    int dot_threshold_;
    bool fallback_to_any_marker_;
    int dot_radius_;
    int arrow_thickness_;
    int arrow_shadow_thickness_;
    double arrow_tip_length_;
    std::string active_camera_;
    bool guide_mode_enabled_;
    std::string mode_;
    bool display_detection_markers_;
    bool display_three_axes_;
    bool display_marker_id_;
    double distance_font_scale_;

    // ArUco parameters
    std::string aruco_dictionary_;
    double aruco_min_marker_perimeter_rate_;
    double aruco_adaptive_thresh_constant_;

    // Hardcoded
    const int process_every_n_frames_ = 1;
    const int jpeg_quality_ = 100;

    cv::Mat camera_matrix_, dist_coeffs_;
    std::mutex cam_mutex_;

    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
    image_transport::Subscriber image_sub_;
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr guide_mode_sub_;

    int frame_counter_;
    bool is_processing_params_;
    std::chrono::steady_clock::time_point last_param_update_;
    const int param_debounce_ms_ = 100;

    void declare_parameters();
    void load_parameters(const std::set<std::string>& skip_params = {});
    void update_aruco_params();
    void update_camera_subscription();
    void caminfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void process_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg, bool show_visuals);
    std::tuple<int, cv::Vec3d, cv::Vec3d> select_guidance_target(const std::vector<int>& ids,
                                                                 const std::vector<cv::Vec3d>& rvecs,
                                                                 const std::vector<cv::Vec3d>& tvecs);
    void draw_guidance(cv::Mat& frame, const cv::Point& img_center, const cv::Point& center_2d, double distance);
    void publish_pose(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                      const cv::Vec3d& rvec, const cv::Vec3d& tvec);
    void publish_compressed_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const cv::Mat& frame);
    cv::Scalar parse_color(const std::string& color_str);

    
    std::deque<std::pair<cv::Vec3d, cv::Vec3d>> pose_history_; // Pose history
    int undetected_frame_count_ = 0; // Consecutive frames without detection
    int max_consecutive_misses_ = 20; // Maximum allowed misses before resetting
    double alpha_ = 0.9; // Smoothing factor for exponential smoothing
    int max_pose_history_size_ = 10; // Maximum size of pose history
};

#endif // ARROW_GUIDANCE_NODE_HPP_
