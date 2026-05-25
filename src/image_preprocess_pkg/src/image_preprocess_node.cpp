#include <algorithm>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sentinel_interfaces/msg/frame_info.hpp"

class ImagePreprocessNode : public rclcpp::Node
{
public:
    ImagePreprocessNode()
    : Node("image_preprocess_node")
    {
        declare_parameter<std::string>("eo_image_topic", "/camera/eo");
        declare_parameter<std::string>("eo_frame_info_topic", "/camera/eo/frame_info");
        declare_parameter<std::string>("eo_output_topic", "/video/eo/preprocessed");
        declare_parameter<std::string>(
            "eo_output_frame_info_topic",
            "/video/eo/preprocessed/frame_info");
        declare_parameter<int>("sync_queue_size", 30);
        declare_parameter<int>("eo_width", 1280);
        declare_parameter<int>("eo_height", 720);
        declare_parameter<bool>("enable_calibration", false);
        declare_parameter<bool>("eo_flip_vertical", false);
        declare_parameter<int>("calibration_width", 640);
        declare_parameter<int>("calibration_height", 480);
        declare_parameter<double>("camera_fx", 0.0);
        declare_parameter<double>("camera_fy", 0.0);
        declare_parameter<double>("camera_cx", 0.0);
        declare_parameter<double>("camera_cy", 0.0);
        declare_parameter<std::vector<double>>(
            "dist_coeffs",
            std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});

        sync_queue_size_ = std::max(1, static_cast<int>(get_parameter("sync_queue_size").as_int()));
        expected_width_ = static_cast<uint32_t>(
            std::max<int64_t>(1, get_parameter("eo_width").as_int()));
        expected_height_ = static_cast<uint32_t>(
            std::max<int64_t>(1, get_parameter("eo_height").as_int()));
        calibration_width_ = static_cast<uint32_t>(
            std::max<int64_t>(1, get_parameter("calibration_width").as_int()));
        calibration_height_ = static_cast<uint32_t>(
            std::max<int64_t>(1, get_parameter("calibration_height").as_int()));
        eo_flip_vertical_ = get_parameter("eo_flip_vertical").as_bool();

        eo_pipeline_.image_topic = get_parameter("eo_image_topic").as_string();
        eo_pipeline_.frame_info_topic = get_parameter("eo_frame_info_topic").as_string();
        eo_pipeline_.output_topic = get_parameter("eo_output_topic").as_string();
        eo_pipeline_.output_frame_info_topic =
            get_parameter("eo_output_frame_info_topic").as_string();

        load_calibration();
        parameter_callback_handle_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & parameters) {
                return handle_parameter_update(parameters);
            });
        setup_pipeline();

        RCLCPP_INFO(get_logger(), "EO ImagePreprocessNode started");
        RCLCPP_INFO(get_logger(), "EO input image  : %s", eo_pipeline_.image_topic.c_str());
        RCLCPP_INFO(get_logger(), "EO input info   : %s", eo_pipeline_.frame_info_topic.c_str());
        RCLCPP_INFO(get_logger(), "EO output image : %s", eo_pipeline_.output_topic.c_str());
        RCLCPP_INFO(
            get_logger(),
            "EO output info  : %s",
            eo_pipeline_.output_frame_info_topic.c_str());
        RCLCPP_INFO(
            get_logger(),
            "EO expected size: %ux%u",
            expected_width_,
            expected_height_);
        RCLCPP_INFO(
            get_logger(),
            "EO vertical flip: %s",
            eo_flip_vertical_ ? "enabled" : "disabled");
    }

private:
    struct Pipeline
    {
        std::string image_topic;
        std::string frame_info_topic;
        std::string output_topic;
        std::string output_frame_info_topic;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        rclcpp::Subscription<sentinel_interfaces::msg::FrameInfo>::SharedPtr frame_info_sub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
        rclcpp::Publisher<sentinel_interfaces::msg::FrameInfo>::SharedPtr frame_info_pub;
        std::unordered_map<int64_t, sensor_msgs::msg::Image::SharedPtr> image_buffer;
        std::unordered_map<int64_t, sentinel_interfaces::msg::FrameInfo::SharedPtr>
            frame_info_buffer;
        uint64_t input_images = 0;
        uint64_t output_images = 0;
    };

    void load_calibration()
    {
        const bool enable_calibration = get_parameter("enable_calibration").as_bool();
        const double fx = get_parameter("camera_fx").as_double();
        const double fy = get_parameter("camera_fy").as_double();
        const double cx = get_parameter("camera_cx").as_double();
        const double cy = get_parameter("camera_cy").as_double();
        const auto dist_coeffs = get_parameter("dist_coeffs").as_double_array();

        calibration_configured_ = (fx != 0.0 && fy != 0.0 && cx != 0.0 && cy != 0.0);
        if (!calibration_configured_) {
            calibration_enabled_ = false;
            RCLCPP_WARN(get_logger(), "EO calibration disabled: camera_fx/fy/cx/cy not set");
            return;
        }

        base_camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0);
        dist_coeffs_ = cv::Mat(dist_coeffs).clone();
        calibration_enabled_ = enable_calibration;

        if (!calibration_enabled_) {
            RCLCPP_INFO(get_logger(), "EO calibration disabled by enable_calibration=false");
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "EO calibration enabled: source=%ux%u configured target=%ux%u "
            "fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
            calibration_width_,
            calibration_height_,
            expected_width_,
            expected_height_,
            fx,
            fy,
            cx,
            cy);
    }

    rcl_interfaces::msg::SetParametersResult handle_parameter_update(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & parameter : parameters) {
            if (parameter.get_name() == "enable_calibration") {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                    result.successful = false;
                    result.reason = "enable_calibration must be a bool";
                    return result;
                }

                const bool requested = parameter.as_bool();
                if (requested && !calibration_configured_) {
                    result.successful = false;
                    result.reason = "camera_fx/fy/cx/cy must be set before enabling calibration";
                    return result;
                }

                calibration_enabled_ = requested;
                RCLCPP_INFO(
                    get_logger(),
                    "EO calibration %s by parameter update",
                    calibration_enabled_ ? "enabled" : "disabled");
                continue;
            }

            if (parameter.get_name() == "eo_flip_vertical") {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                    result.successful = false;
                    result.reason = "eo_flip_vertical must be a bool";
                    return result;
                }

                eo_flip_vertical_ = parameter.as_bool();
                RCLCPP_INFO(
                    get_logger(),
                    "EO vertical flip %s by parameter update",
                    eo_flip_vertical_ ? "enabled" : "disabled");
            }
        }

        return result;
    }

    void setup_pipeline()
    {
        eo_pipeline_.image_pub =
            create_publisher<sensor_msgs::msg::Image>(eo_pipeline_.output_topic, 10);
        eo_pipeline_.frame_info_pub =
            create_publisher<sentinel_interfaces::msg::FrameInfo>(
                eo_pipeline_.output_frame_info_topic,
                10);

        eo_pipeline_.image_sub = create_subscription<sensor_msgs::msg::Image>(
            eo_pipeline_.image_topic,
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                handle_image(std::move(msg));
            });

        eo_pipeline_.frame_info_sub =
            create_subscription<sentinel_interfaces::msg::FrameInfo>(
                eo_pipeline_.frame_info_topic,
                rclcpp::SensorDataQoS(),
                [this](sentinel_interfaces::msg::FrameInfo::SharedPtr msg) {
                    handle_frame_info(std::move(msg));
                });
    }

    void handle_image(sensor_msgs::msg::Image::SharedPtr msg)
    {
        eo_pipeline_.input_images++;
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "EO calibration input [%s]: %ux%u encoding=%s count=%lu",
            eo_pipeline_.image_topic.c_str(),
            msg->width,
            msg->height,
            msg->encoding.c_str(),
            eo_pipeline_.input_images);

        if (msg->width != expected_width_ || msg->height != expected_height_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "EO image size mismatch: expected=%ux%u actual=%ux%u",
                expected_width_,
                expected_height_,
                msg->width,
                msg->height);
        }

        const auto stamp_ns = header_to_ns(msg->header);
        if (stamp_ns < 0) {
            RCLCPP_WARN(get_logger(), "Received EO image without a valid stamp");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            eo_pipeline_.image_buffer[stamp_ns] = msg;
            trim_buffer(eo_pipeline_.image_buffer);
        }
        try_process_pair(stamp_ns);
    }

    void handle_frame_info(sentinel_interfaces::msg::FrameInfo::SharedPtr msg)
    {
        const auto stamp_ns = stamp_to_ns(msg->stamp);
        if (stamp_ns < 0) {
            RCLCPP_WARN(get_logger(), "Received EO FrameInfo without a valid stamp");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            eo_pipeline_.frame_info_buffer[stamp_ns] = msg;
            trim_buffer(eo_pipeline_.frame_info_buffer);
        }
        try_process_pair(stamp_ns);
    }

    void try_process_pair(int64_t stamp_ns)
    {
        sensor_msgs::msg::Image::SharedPtr image_msg;
        sentinel_interfaces::msg::FrameInfo::SharedPtr frame_info_msg;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto image_it = eo_pipeline_.image_buffer.find(stamp_ns);
            auto frame_it = eo_pipeline_.frame_info_buffer.find(stamp_ns);
            if (image_it == eo_pipeline_.image_buffer.end() ||
                frame_it == eo_pipeline_.frame_info_buffer.end())
            {
                return;
            }

            image_msg = std::move(image_it->second);
            frame_info_msg = std::move(frame_it->second);
            eo_pipeline_.image_buffer.erase(image_it);
            eo_pipeline_.frame_info_buffer.erase(frame_it);
        }

        process_and_publish(image_msg, frame_info_msg);
    }

    void process_and_publish(
        const sensor_msgs::msg::Image::SharedPtr & image_msg,
        const sentinel_interfaces::msg::FrameInfo::SharedPtr & frame_info_msg)
    {
        try {
            const auto start = std::chrono::steady_clock::now();
            cv::Mat input = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
            cv::Mat calibrated = calibrate(input);
            cv::Mat processed = flip_vertical(calibrated);
            const auto elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();

            auto output_image =
                cv_bridge::CvImage(image_msg->header, "bgr8", processed).toImageMsg();
            eo_pipeline_.image_pub->publish(*output_image);

            auto output_frame_info = *frame_info_msg;
            output_frame_info.width = static_cast<uint32_t>(processed.cols);
            output_frame_info.height = static_cast<uint32_t>(processed.rows);
            eo_pipeline_.frame_info_pub->publish(output_frame_info);

            eo_pipeline_.output_images++;
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "EO preprocess output [%s]: %dx%d count=%lu calibration_flip=%.2fms flip_vertical=%s",
                eo_pipeline_.output_topic.c_str(),
                processed.cols,
                processed.rows,
                eo_pipeline_.output_images,
                elapsed_ms,
                eo_flip_vertical_ ? "true" : "false");
        } catch (const std::exception & e) {
            RCLCPP_ERROR(
                get_logger(),
                "Failed EO calibration frame on %s: %s",
                eo_pipeline_.output_topic.c_str(),
                e.what());
        }
    }

    cv::Mat calibrate(const cv::Mat & input)
    {
        if (!calibration_enabled_) {
            return input;
        }

        cv::Mat calibrated;
        cv::undistort(input, calibrated, scaled_camera_matrix(input.cols, input.rows), dist_coeffs_);
        return calibrated;
    }

    cv::Mat flip_vertical(const cv::Mat & input) const
    {
        if (!eo_flip_vertical_) {
            return input;
        }

        cv::Mat flipped;
        cv::flip(input, flipped, 0);
        return flipped;
    }

    cv::Mat scaled_camera_matrix(int image_width, int image_height) const
    {
        const double scale_x =
            static_cast<double>(image_width) / static_cast<double>(calibration_width_);
        const double scale_y =
            static_cast<double>(image_height) / static_cast<double>(calibration_height_);

        cv::Mat camera_matrix = base_camera_matrix_.clone();
        camera_matrix.at<double>(0, 0) *= scale_x;
        camera_matrix.at<double>(0, 2) *= scale_x;
        camera_matrix.at<double>(1, 1) *= scale_y;
        camera_matrix.at<double>(1, 2) *= scale_y;
        return camera_matrix;
    }

    template<typename T>
    void trim_buffer(std::unordered_map<int64_t, T> & buffer)
    {
        while (static_cast<int>(buffer.size()) > sync_queue_size_) {
            auto oldest = std::min_element(
                buffer.begin(),
                buffer.end(),
                [](const auto & a, const auto & b) { return a.first < b.first; });
            buffer.erase(oldest);
        }
    }

    static int64_t header_to_ns(const std_msgs::msg::Header & header)
    {
        return stamp_to_ns(header.stamp);
    }

    static int64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp)
    {
        return static_cast<int64_t>(stamp.sec) * 1000000000LL +
            static_cast<int64_t>(stamp.nanosec);
    }

    std::mutex mutex_;
    Pipeline eo_pipeline_;
    int sync_queue_size_{30};
    uint32_t expected_width_{1280};
    uint32_t expected_height_{720};
    uint32_t calibration_width_{640};
    uint32_t calibration_height_{480};
    bool calibration_configured_{false};
    bool calibration_enabled_{false};
    bool eo_flip_vertical_{false};
    cv::Mat base_camera_matrix_;
    cv::Mat dist_coeffs_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        parameter_callback_handle_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePreprocessNode>());
    rclcpp::shutdown();
    return 0;
}
