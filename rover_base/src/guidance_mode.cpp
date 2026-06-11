
#include "rover_base/guidance_mode.hpp"

ArrowGuidanceNode::ArrowGuidanceNode() : Node("guidance_node"), frame_counter_(0), is_processing_params_(false) {
    declare_parameters();
    load_parameters();

    compressed_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "/camera/guidance_image/compressed", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/aruco_marker/pose", 10);

    aruco_dict_ = cv::aruco::getPredefinedDictionary(dictionary_from_string(aruco_dictionary_));
    aruco_params_ = cv::aruco::DetectorParameters::create();
    update_aruco_params();

    camera_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/set_active_camera", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == "front" || msg->data == "back") {
                if (msg->data != active_camera_) {
                    active_camera_ = msg->data;
                    update_camera_subscription();
                    RCLCPP_INFO(this->get_logger(), "Active camera set to: %s (via /set_active_camera)", active_camera_.c_str());
                    this->set_parameter(rclcpp::Parameter("active_camera", active_camera_));
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid camera value on /set_active_camera: %s", msg->data.c_str());
            }
        }
    );

    guide_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/set_guide_mode", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data != guide_mode_enabled_) {
                guide_mode_enabled_ = msg->data;
                frame_counter_ = 0;
                RCLCPP_INFO(this->get_logger(), "Guide mode set to: %s (via /set_guide_mode)", guide_mode_enabled_ ? "ENABLED" : "DISABLED");
                this->set_parameter(rclcpp::Parameter("guide_mode_enabled", guide_mode_enabled_));
            }
        }
    );

    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
            if (is_processing_params_) {
                RCLCPP_WARN(this->get_logger(), "Skipping reentrant parameter callback");
                return rcl_interfaces::msg::SetParametersResult().set__successful(false);
            }
            auto now = std::chrono::steady_clock::now();
            auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_param_update_).count();
            if (time_since_last < param_debounce_ms_) {
                RCLCPP_WARN(this->get_logger(), "Parameter update debounced: %ld ms since last update", time_since_last);
                return rcl_interfaces::msg::SetParametersResult().set__successful(false);
            }

            is_processing_params_ = true;
            last_param_update_ = now;

            std::set<std::string> skip_params;
            bool camera_updated = false;
            bool visualization_updated = false;
            bool aruco_updated = false;
            for (const auto& param : params) {
                if (param.get_name() == "active_camera") {
                    active_camera_ = param.as_string();
                    camera_updated = true;
                    skip_params.insert("active_camera");
                }
                if (param.get_name() == "guide_mode_enabled") {
                    guide_mode_enabled_ = param.as_bool();
                    visualization_updated = true;
                    skip_params.insert("guide_mode_enabled");
                }
                if (param.get_name() == "mode") {
                    mode_ = param.as_string();
                    visualization_updated = true;
                    skip_params.insert("mode");
                }
                if (param.get_name() == "arrow_color") {
                    arrow_color_ = param.as_string();
                    visualization_updated = true;
                    skip_params.insert("arrow_color");
                }
                if (param.get_name() == "fallback_to_any_marker") {
                    fallback_to_any_marker_ = param.as_bool();
                    skip_params.insert("fallback_to_any_marker");
                }
                if (param.get_name() == "arrow_thickness" || param.get_name() == "arrow_shadow_thickness" ||
                    param.get_name() == "dot_radius" || param.get_name() == "dot_threshold" ||
                    param.get_name() == "display_detection_markers" || param.get_name() == "display_three_axes" ||
                    param.get_name() == "display_marker_id" || param.get_name() == "distance_font_scale") {
                    visualization_updated = true;
                    skip_params.insert(param.get_name());
                }
                if (param.get_name().rfind("aruco_", 0) == 0) {
                    aruco_updated = true;
                    skip_params.insert(param.get_name());
                }
            }
            if (camera_updated) update_camera_subscription();
            if (visualization_updated) frame_counter_ = 0;
            load_parameters(skip_params);
            if (aruco_updated) {
                aruco_dict_ = cv::aruco::getPredefinedDictionary(dictionary_from_string(aruco_dictionary_));
                update_aruco_params();
            }
            is_processing_params_ = false;
            return rcl_interfaces::msg::SetParametersResult().set__successful(true);
        });
}

void ArrowGuidanceNode::initialize() {
    image_transport_ = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());
    update_camera_subscription();
    RCLCPP_INFO(this->get_logger(), "ArrowGuidanceNode initialized.");
}

void ArrowGuidanceNode::declare_parameters() {
    this->declare_parameter("aruco_marker_size", 0.1);
    this->declare_parameter("arrow_color", "orange");
    this->declare_parameter("target_marker_ids", std::vector<int64_t>{44});
    this->declare_parameter("dot_threshold", 20);
    this->declare_parameter("fallback_to_any_marker", false);
    this->declare_parameter("dot_radius", 10);
    this->declare_parameter("arrow_thickness", 6);
    this->declare_parameter("arrow_shadow_thickness", 14);
    this->declare_parameter("arrow_tip_length", 0.25);
    this->declare_parameter("active_camera", "front");
    this->declare_parameter("guide_mode_enabled", true);
    this->declare_parameter("mode", "single");
    this->declare_parameter("display_detection_markers", true);
    this->declare_parameter("display_three_axes", false);
    this->declare_parameter("display_marker_id", false);
    this->declare_parameter("distance_font_scale", 0.5);

    this->declare_parameter("aruco_dictionary", "DICT_4X4_50");
    this->declare_parameter("aruco_min_marker_perimeter_rate", 0.03);
    this->declare_parameter("aruco_adaptive_thresh_constant", 7.0);
}

void ArrowGuidanceNode::load_parameters(const std::set<std::string>& skip_params) {
    if (skip_params.find("aruco_marker_size") == skip_params.end()) {
        aruco_marker_size_ = this->get_parameter("aruco_marker_size").as_double();
        if (aruco_marker_size_ <= 0) {
            aruco_marker_size_ = 0.1;
        }
    }
    if (skip_params.find("arrow_color") == skip_params.end()) {
        arrow_color_ = this->get_parameter("arrow_color").as_string();
    }
    if (skip_params.find("target_marker_ids") == skip_params.end()) {
        target_marker_ids_ = this->get_parameter("target_marker_ids").as_integer_array();
    }
    if (skip_params.find("dot_threshold") == skip_params.end()) {
        dot_threshold_ = this->get_parameter("dot_threshold").as_int();
        if (dot_threshold_ < 0) {
            dot_threshold_ = 20;
        }
    }
    if (skip_params.find("fallback_to_any_marker") == skip_params.end()) {
        fallback_to_any_marker_ = this->get_parameter("fallback_to_any_marker").as_bool();
    }
    if (skip_params.find("dot_radius") == skip_params.end()) {
        dot_radius_ = this->get_parameter("dot_radius").as_int();
        if (dot_radius_ < 1) {
            dot_radius_ = 10;
        }
    }
    if (skip_params.find("arrow_thickness") == skip_params.end()) {
        arrow_thickness_ = this->get_parameter("arrow_thickness").as_int();
        if (arrow_thickness_ < 1) {
            arrow_thickness_ = 6;
        }
    }
    if (skip_params.find("arrow_shadow_thickness") == skip_params.end()) {
        arrow_shadow_thickness_ = this->get_parameter("arrow_shadow_thickness").as_int();
        if (arrow_shadow_thickness_ < 1) {
            arrow_shadow_thickness_ = 14;
        }
    }
    if (skip_params.find("arrow_tip_length") == skip_params.end()) {
        arrow_tip_length_ = this->get_parameter("arrow_tip_length").as_double();
        if (arrow_tip_length_ <= 0) {
            arrow_tip_length_ = 0.25;
        }
    }
    if (skip_params.find("active_camera") == skip_params.end())
        active_camera_ = this->get_parameter("active_camera").as_string();
    if (skip_params.find("guide_mode_enabled") == skip_params.end())
        guide_mode_enabled_ = this->get_parameter("guide_mode_enabled").as_bool();
    if (skip_params.find("mode") == skip_params.end()) {
        mode_ = this->get_parameter("mode").as_string();
        if (mode_ != "single" && mode_ != "dual_center") {
            RCLCPP_WARN(this->get_logger(), "Invalid mode '%s', defaulting to 'single'", mode_.c_str());
            mode_ = "single";
        }
    }
    if (skip_params.find("display_detection_markers") == skip_params.end())
        display_detection_markers_ = this->get_parameter("display_detection_markers").as_bool();
    if (skip_params.find("display_three_axes") == skip_params.end())
        display_three_axes_ = this->get_parameter("display_three_axes").as_bool();
    if (skip_params.find("display_marker_id") == skip_params.end())
        display_marker_id_ = this->get_parameter("display_marker_id").as_bool();
    if (skip_params.find("distance_font_scale") == skip_params.end())
        distance_font_scale_ = this->get_parameter("distance_font_scale").as_double();

    if (skip_params.find("aruco_dictionary") == skip_params.end())
        aruco_dictionary_ = this->get_parameter("aruco_dictionary").as_string();
    if (skip_params.find("aruco_min_marker_perimeter_rate") == skip_params.end())
        aruco_min_marker_perimeter_rate_ = this->get_parameter("aruco_min_marker_perimeter_rate").as_double();
    if (skip_params.find("aruco_adaptive_thresh_constant") == skip_params.end())
        aruco_adaptive_thresh_constant_ = this->get_parameter("aruco_adaptive_thresh_constant").as_double();
}

void ArrowGuidanceNode::update_aruco_params() {
    aruco_params_->minMarkerPerimeterRate = aruco_min_marker_perimeter_rate_;
    aruco_params_->adaptiveThreshConstant = aruco_adaptive_thresh_constant_;
}

void ArrowGuidanceNode::update_camera_subscription() {
    if (!image_transport_) return;
    if (image_sub_) image_sub_.shutdown();
    if (caminfo_sub_) caminfo_sub_.reset();

    std::string image_topic, info_topic;
    if (active_camera_ == "front") {
        image_topic = "/motion/front_camera/color/image_raw";
        info_topic = "/motion/front_camera/color/camera_info";
    } else {
        image_topic = "/motion/back_camera/color/image_raw";
        info_topic = "/motion/back_camera/color/camera_info";
    }

    image_sub_ = image_transport_->subscribe(
        image_topic, 10,
        std::bind(&ArrowGuidanceNode::image_callback, this, std::placeholders::_1)
    );

    caminfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic, 10,
        std::bind(&ArrowGuidanceNode::caminfo_callback, this, std::placeholders::_1)
    );
}

void ArrowGuidanceNode::caminfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cam_mutex_);
    if (msg->k.size() != 9 || msg->d.empty()) {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info received");
        return;
    }
    camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<void*>(static_cast<const void*>(msg->k.data()))).clone();
    dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<void*>(static_cast<const void*>(msg->d.data()))).clone();
}

void ArrowGuidanceNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    process_image(msg, guide_mode_enabled_);
}

void ArrowGuidanceNode::process_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg, bool show_visuals) {
    cv::Mat camera_matrix, dist_coeffs;
    {
        std::lock_guard<std::mutex> lock(cam_mutex_);
        if (camera_matrix_.empty() || dist_coeffs_.empty()) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Camera calibration data not available");
            return;
        }
        camera_matrix = camera_matrix_.clone();
        dist_coeffs = dist_coeffs_.clone();
    }

    frame_counter_++;
    if (frame_counter_ % process_every_n_frames_ != 0) {
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat frame = cv_ptr->image;
    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty image");
        return;
    }

    cv::Mat output_frame = frame.clone();
    cv::Point img_center(frame.cols / 2, frame.rows / 2);

    if (show_visuals) {
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, aruco_params_);

        if (!ids.empty()) {
            // Reset the undetected frame counter
            undetected_frame_count_ = 0;

            // Draw detected markers if enabled
            if (display_detection_markers_) {
                if (display_marker_id_) {
                    cv::aruco::drawDetectedMarkers(output_frame, corners, ids, cv::Scalar(0, 255, 0));
                } else {
                    cv::aruco::drawDetectedMarkers(output_frame, corners, std::vector<int>{}, cv::Scalar(0, 255, 0));
                }
            }

            // Estimate pose
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, aruco_marker_size_, camera_matrix, dist_coeffs, rvecs, tvecs);
            auto [best_idx, best_rvec, best_tvec] = select_guidance_target(ids, rvecs, tvecs);

            if (best_idx >= 0) {
                // Update pose history
                pose_history_.push_back(std::make_pair(best_rvec, best_tvec));
                if (pose_history_.size() > max_pose_history_size_) {
                    pose_history_.pop_front();
                }

                // Publish pose
                publish_pose(msg, best_rvec, best_tvec);

                // Draw guidance
                std::vector<cv::Point3f> obj_points = {cv::Point3f(0, 0, 0)};
                std::vector<cv::Point2f> img_points;
                cv::projectPoints(obj_points, best_rvec, best_tvec, camera_matrix, dist_coeffs, img_points);
                cv::Point center_2d = img_points[0];
                if (center_2d.x >= 0 && center_2d.x < frame.cols && center_2d.y >= 0 && center_2d.y < frame.rows) {
                    double distance = cv::norm(best_tvec);
                    draw_guidance(output_frame, img_center, center_2d, distance);
                }

                if (display_three_axes_) {
                    std::vector<cv::Point3f> axis_points = {
                        cv::Point3f(0, 0, 0), cv::Point3f(0.1, 0, 0), cv::Point3f(0, 0.1, 0), cv::Point3f(0, 0, 0.1)
                    };
                    std::vector<cv::Point2f> projected_axes;
                    cv::projectPoints(axis_points, best_rvec, best_tvec, camera_matrix, dist_coeffs, projected_axes);
                    cv::Point2i p0(projected_axes[0]), px(projected_axes[1]), py(projected_axes[2]), pz(projected_axes[3]);
                    cv::line(output_frame, p0, px, cv::Scalar(0, 0, 255), 3); // X-axis (red)
                    cv::line(output_frame, p0, py, cv::Scalar(0, 255, 0), 3); // Y-axis (green)
                    cv::line(output_frame, p0, pz, cv::Scalar(255, 0, 0), 3); // Z-axis (blue)
                }
            } else {
                // No valid target found
                cv::circle(output_frame, img_center, dot_radius_, cv::Scalar(0, 0, 255), -1); // Red dot
                cv::putText(output_frame, "No valid target", cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX, distance_font_scale_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            }
        } else {
            // Marker not detected
            undetected_frame_count_++;

            if (undetected_frame_count_ <= max_consecutive_misses_) {
                // Use exponential smoothing with the last known pose
                if (!pose_history_.empty()) {
                    const auto& [last_rvec, last_tvec] = pose_history_.back();

                    // Smooth the pose
                    cv::Vec3d smoothed_rvec = alpha_ * last_rvec + (1 - alpha_) * last_rvec;
                    cv::Vec3d smoothed_tvec = alpha_ * last_tvec + (1 - alpha_) * last_tvec;

                    // Publish smoothed pose
                    publish_pose(msg, smoothed_rvec, smoothed_tvec);

                    // Draw guidance based on smoothed pose
                    std::vector<cv::Point3f> obj_points = {cv::Point3f(0, 0, 0)};
                    std::vector<cv::Point2f> img_points;
                    cv::projectPoints(obj_points, smoothed_rvec, smoothed_tvec, camera_matrix, dist_coeffs, img_points);
                    cv::Point center_2d = img_points[0];
                    if (center_2d.x >= 0 && center_2d.x < frame.cols && center_2d.y >= 0 && center_2d.y < frame.rows) {
                        double distance = cv::norm(smoothed_tvec);
                        draw_guidance(output_frame, img_center, center_2d, distance);
                    }
                } else {
                    // No history available, show default message
                    cv::circle(output_frame, img_center, dot_radius_, cv::Scalar(0, 0, 255), -1); // Red dot
                    cv::putText(output_frame, "No markers detected", cv::Point(10, 30),
                                cv::FONT_HERSHEY_SIMPLEX, distance_font_scale_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                }
            } else {
                // Too many consecutive misses, reset pose history
                pose_history_.clear();
                undetected_frame_count_ = 0;

                // Show reset message
                cv::circle(output_frame, img_center, dot_radius_, cv::Scalar(0, 0, 255), -1); // Red dot
                cv::putText(output_frame, "Resetting pose history", cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX, distance_font_scale_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            }
        }
    } else {
        cv::circle(output_frame, img_center, dot_radius_, cv::Scalar(0, 0, 255), -1); // Red dot
        cv::putText(output_frame, "Guide mode disabled", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, distance_font_scale_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }

    // Publish the processed/compressed image
    publish_compressed_image(msg, show_visuals ? output_frame : frame);
}

std::tuple<int, cv::Vec3d, cv::Vec3d> ArrowGuidanceNode::select_guidance_target(const std::vector<int>& ids,
                                                                               const std::vector<cv::Vec3d>& rvecs,
                                                                               const std::vector<cv::Vec3d>& tvecs) {
    // Step 1: Try to find target markers based on mode
    if (mode_ == "dual_center") {
        // Dual center mode requires exactly two target IDs
        if (target_marker_ids_.size() != 2) {
            RCLCPP_WARN(this->get_logger(), "Dual center mode requires exactly 2 target IDs, found %zu", target_marker_ids_.size());
            return {-1, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0)};
        }

        std::map<int, size_t> id_to_index;
        for (size_t i = 0; i < ids.size(); ++i) {
            id_to_index[ids[i]] = i;
        }

        // Check if both target IDs are detected
        if (id_to_index.count(target_marker_ids_[0]) && id_to_index.count(target_marker_ids_[1])) {
            size_t idx0 = id_to_index[target_marker_ids_[0]];
            size_t idx1 = id_to_index[target_marker_ids_[1]];
            cv::Vec3d midpoint = (tvecs[idx0] + tvecs[idx1]) * 0.5;
            // Use rotation from the first marker
           // RCLCPP_INFO(this->get_logger(), "Dual center: using markers %ld and %ld", target_marker_ids_[0], target_marker_ids_[1]);
            return {0, rvecs[idx0], midpoint};
        }
        // If not both detected, proceed to fallback
    } else if (mode_ == "single") {
        // Single mode: find the first matching target ID
        for (size_t i = 0; i < ids.size(); ++i) {
            for (int target_id : target_marker_ids_) {
                if (ids[i] == target_id) {
                    //RCLCPP_INFO(this->get_logger(), "Single mode: selected marker %d", target_id);
                    return {static_cast<int>(i), rvecs[i], tvecs[i]};
                }
            }
        }
        // If no target ID found, proceed to fallback
    }

    // Step 2: Apply fallback logic if enabled
    if (fallback_to_any_marker_ && !ids.empty()) {
        double min_dist = std::numeric_limits<double>::max();
        int min_idx = -1;
        for (size_t i = 0; i < tvecs.size(); ++i) {
            double dist = cv::norm(tvecs[i]);
            if (dist < min_dist) {
                min_dist = dist;
                min_idx = static_cast<int>(i);
            }
        }
        if (min_idx >= 0) {
            RCLCPP_INFO(this->get_logger(), "Fallback: using closest marker ID %d", ids[min_idx]);
            return {min_idx, rvecs[min_idx], tvecs[min_idx]};
        }
    }

    // Step 3: No valid target found
    RCLCPP_DEBUG(this->get_logger(), "No target markers found, fallback disabled or no markers available");
    return {-1, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0)};
}

void ArrowGuidanceNode::draw_guidance(cv::Mat& frame, const cv::Point& img_center, const cv::Point& center_2d, double distance) {
    int base = std::min(frame.cols, frame.rows);

    int arrow_thickness = std::max(3, base / 120);
    int shadow_thickness = arrow_thickness + 3;
    int dot_radius = std::max(5, base / 60);
    int glow_radius = dot_radius + 4;
    int ring_radius = dot_radius + 10;
    int ring_thickness = 2;

    int dx = center_2d.x - img_center.x;
    double align_ratio = std::abs(dx) / (frame.cols / 2.0);

    std::string direction;
    cv::Scalar arrow_color, glow_color, ring_color;
    if (align_ratio < 0.07) {
        direction = "STRAIGHT";
        arrow_color = cv::Scalar(60, 180, 75);
        glow_color = cv::Scalar(144, 238, 144);
        ring_color = cv::Scalar(60, 180, 75);
    } else if (dx < 0) {
        direction = "LEFT";
        arrow_color = cv::Scalar(0, 140, 255);
        glow_color = cv::Scalar(255, 204, 153);
        ring_color = cv::Scalar(0, 140, 255);
    } else {
        direction = "RIGHT";
        arrow_color = cv::Scalar(255, 80, 80);
        glow_color = cv::Scalar(255, 204, 229);
        ring_color = cv::Scalar(255, 80, 80);
    }

    cv::arrowedLine(frame, img_center + cv::Point(2,2), center_2d + cv::Point(2,2),
                    cv::Scalar(60,60,60), shadow_thickness, cv::LINE_AA, 0, 0.25);
    cv::arrowedLine(frame, img_center, center_2d, arrow_color, arrow_thickness, cv::LINE_AA, 0, 0.25);

    double pulse = 0.8 + 0.2 * std::sin((cv::getTickCount() / (double)cv::getTickFrequency()) * 2 * M_PI);
    cv::circle(frame, img_center, glow_radius, glow_color, -1, cv::LINE_AA);
    cv::circle(frame, img_center, static_cast<int>(dot_radius * pulse), cv::Scalar(255,255,255), -1, cv::LINE_AA);

    double arc_extent = 360.0 * (1.0 - align_ratio);
    for (int i = 0; i < static_cast<int>(arc_extent); ++i) {
        double theta = i * M_PI / 180.0;
        int x = static_cast<int>(img_center.x + ring_radius * std::cos(theta));
        int y = static_cast<int>(img_center.y + ring_radius * std::sin(theta));
        cv::circle(frame, cv::Point(x, y), ring_thickness/2, ring_color, -1, cv::LINE_AA);
    }

    int card_width = base * 0.38;
    int card_height = base * 0.10;
    int x = 20, y = frame.rows - card_height - 18;
    cv::rectangle(frame, cv::Rect(x, y, card_width, card_height), cv::Scalar(34,40,49), -1, cv::LINE_AA, 0);

    std::ostringstream oss;
    oss << "Distance: " << std::fixed << std::setprecision(2) << distance << " m";
    cv::putText(frame, oss.str(), cv::Point(x + 20, y + card_height/2 + 8),
                cv::FONT_HERSHEY_SIMPLEX, base / 1100.0, cv::Scalar(248,248,248), 1, cv::LINE_AA);

    cv::putText(frame, direction, cv::Point(x + 20, y + card_height/2 + 28),
                cv::FONT_HERSHEY_SIMPLEX, base / 1100.0, ring_color, 2, cv::LINE_AA);
}

void ArrowGuidanceNode::publish_pose(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                                    const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.header.frame_id = "marker_frame";
    pose_msg.pose.position.x = tvec[0];
    pose_msg.pose.position.y = tvec[1];
    pose_msg.pose.position.z = tvec[2];

    cv::Mat rot_mat;
    cv::Rodrigues(rvec, rot_mat);

    tf2::Matrix3x3 tf_rot(rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2),
                          rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2),
                          rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2));
    tf2::Quaternion q;
    tf_rot.getRotation(q);

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_pub_->publish(pose_msg);
}

void ArrowGuidanceNode::publish_compressed_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const cv::Mat& frame) {
    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot publish empty frame");
        return;
    }
    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (!cv::imencode(".PNG", frame, buf, params)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to encode image");
        return;
    }

    auto compressed_msg = sensor_msgs::msg::CompressedImage();
    compressed_msg.header = msg->header;
    compressed_msg.format = "PNG";
    compressed_msg.data.assign(buf.begin(), buf.end());

    compressed_image_pub_->publish(compressed_msg);
}

cv::Scalar ArrowGuidanceNode::parse_color(const std::string& color_str) {
    std::string lower_color = color_str;
    std::transform(lower_color.begin(), lower_color.end(), lower_color.begin(), ::tolower);
    if (lower_color == "blue") return cv::Scalar(255, 0, 0);
    if (lower_color == "orange") return cv::Scalar(0, 165, 255);
    if (lower_color == "light_blue") return cv::Scalar(255, 200, 0);
    if (lower_color == "green") return cv::Scalar(0, 255, 0);
    if (lower_color == "red") return cv::Scalar(0, 0, 255);
    if (lower_color == "white") return cv::Scalar(255, 255, 255);
    if (lower_color == "yellow") return cv::Scalar(0, 255, 255);
    if (lower_color == "purple") return cv::Scalar(128, 0, 128);
    try {
        std::vector<int> parts;
        std::stringstream ss(lower_color);
        std::string part;
        while (std::getline(ss, part, ',')) {
            parts.push_back(std::stoi(part));
        }
        if (parts.size() == 3 && std::all_of(parts.begin(), parts.end(), [](int x) { return x >= 0 && x <= 255; })) {
            return cv::Scalar(parts[0], parts[1], parts[2]);
        }
    } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Invalid color format '%s', defaulting to orange", color_str.c_str());
    }
    return cv::Scalar(0, 165, 255); // Default: orange
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArrowGuidanceNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
