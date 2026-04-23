/*
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
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
        declare_parameter<std::string>("ir_image_topic", "/camera/ir");
        declare_parameter<std::string>("ir_frame_info_topic", "/camera/ir/frame_info");
        declare_parameter<std::string>("ir_output_topic", "/video/ir/preprocessed");
        declare_parameter<std::string>("ir_output_frame_info_topic", "/video/ir/preprocessed/frame_info");
        declare_parameter<std::string>("eo_image_topic", "/camera/eo");
        declare_parameter<std::string>("eo_frame_info_topic", "/camera/eo/frame_info");
        declare_parameter<std::string>("eo_output_topic", "/video/eo/preprocessed");
        declare_parameter<std::string>("eo_output_frame_info_topic", "/video/eo/preprocessed/frame_info");
        declare_parameter<int>("sync_queue_size", 30);
        declare_parameter<int>("denoise_kernel_size", 5);
        declare_parameter<double>("clahe_clip_limit", 2.0);
        declare_parameter<int>("clahe_tile_size", 8);

        sync_queue_size_ = std::max(1, static_cast<int>(get_parameter("sync_queue_size").as_int()));
        denoise_kernel_size_ = normalize_kernel_size(static_cast<int>(get_parameter("denoise_kernel_size").as_int()));
        clahe_clip_limit_ = get_parameter("clahe_clip_limit").as_double();
        clahe_tile_size_ = std::max(1, static_cast<int>(get_parameter("clahe_tile_size").as_int()));

        ir_pipeline_ = make_pipeline(
            get_parameter("ir_image_topic").as_string(),
            get_parameter("ir_frame_info_topic").as_string(),
            get_parameter("ir_output_topic").as_string(),
            get_parameter("ir_output_frame_info_topic").as_string()
        );

        eo_pipeline_ = make_pipeline(
            get_parameter("eo_image_topic").as_string(),
            get_parameter("eo_frame_info_topic").as_string(),
            get_parameter("eo_output_topic").as_string(),
            get_parameter("eo_output_frame_info_topic").as_string()
        );

        setup_pipeline(ir_pipeline_);
        setup_pipeline(eo_pipeline_);

        RCLCPP_INFO(get_logger(), "ImagePreprocessNode started");
        log_pipeline("IR", ir_pipeline_);
        log_pipeline("EO", eo_pipeline_);
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
        std::unordered_map<int64_t, sentinel_interfaces::msg::FrameInfo::SharedPtr> frame_info_buffer;
        uint64_t input_images = 0;
        uint64_t output_images = 0;
    };

    static int normalize_kernel_size(int size)
    {
        size = std::max(1, size);
        if (size % 2 == 0) {
            size += 1;
        }
        return size;
    }

    Pipeline make_pipeline(
        const std::string & image_topic,
        const std::string & frame_info_topic,
        const std::string & output_topic,
        const std::string & output_frame_info_topic)
    {
        Pipeline pipeline;
        pipeline.image_topic = image_topic;
        pipeline.frame_info_topic = frame_info_topic;
        pipeline.output_topic = output_topic;
        pipeline.output_frame_info_topic = output_frame_info_topic;
        return pipeline;
    }

    void setup_pipeline(Pipeline & pipeline)
    {
        pipeline.image_pub = create_publisher<sensor_msgs::msg::Image>(pipeline.output_topic, 10);
        pipeline.frame_info_pub = create_publisher<sentinel_interfaces::msg::FrameInfo>(
            pipeline.output_frame_info_topic,
            10
        );

        pipeline.image_sub = create_subscription<sensor_msgs::msg::Image>(
            pipeline.image_topic,
            rclcpp::SensorDataQoS(),
            [this, &pipeline](sensor_msgs::msg::Image::SharedPtr msg) {
                handle_image(pipeline, std::move(msg));
            }
        );

        pipeline.frame_info_sub = create_subscription<sentinel_interfaces::msg::FrameInfo>(
            pipeline.frame_info_topic,
            rclcpp::SensorDataQoS(),
            [this, &pipeline](sentinel_interfaces::msg::FrameInfo::SharedPtr msg) {
                handle_frame_info(pipeline, std::move(msg));
            }
        );
    }

    void handle_image(Pipeline & pipeline, sensor_msgs::msg::Image::SharedPtr msg)
    {
        pipeline.input_images++;
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "preprocess input [%s]: %ux%u encoding=%s count=%lu",
            pipeline.image_topic.c_str(),
            msg->width,
            msg->height,
            msg->encoding.c_str(),
            pipeline.input_images);

        const auto stamp_ns = header_to_ns(msg->header);
        if (stamp_ns < 0) {
            RCLCPP_WARN(get_logger(), "Received image without a valid stamp on %s", pipeline.image_topic.c_str());
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            pipeline.image_buffer[stamp_ns] = msg;
            trim_buffer(pipeline.image_buffer);
        }
        try_process_pair(pipeline, stamp_ns);
    }

    void handle_frame_info(Pipeline & pipeline, sentinel_interfaces::msg::FrameInfo::SharedPtr msg)
    {
        const auto stamp_ns = stamp_to_ns(msg->stamp);
        if (stamp_ns < 0) {
            RCLCPP_WARN(
                get_logger(),
                "Received FrameInfo without a valid stamp on %s",
                pipeline.frame_info_topic.c_str()
            );
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            pipeline.frame_info_buffer[stamp_ns] = msg;
            trim_buffer(pipeline.frame_info_buffer);
        }
        try_process_pair(pipeline, stamp_ns);
    }

    // 같은 타임스탬프를 가진 이미지와 프레임 정보를 찾아 처리
    void try_process_pair(Pipeline & pipeline, int64_t stamp_ns)
    {
        sensor_msgs::msg::Image::SharedPtr image_msg;
        sentinel_interfaces::msg::FrameInfo::SharedPtr frame_info_msg;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto image_it = pipeline.image_buffer.find(stamp_ns);
            auto frame_it = pipeline.frame_info_buffer.find(stamp_ns);
            if (image_it == pipeline.image_buffer.end() || frame_it == pipeline.frame_info_buffer.end()) {
                return;
            }

            image_msg = std::move(image_it->second);
            frame_info_msg = std::move(frame_it->second);
            pipeline.image_buffer.erase(image_it);
            pipeline.frame_info_buffer.erase(frame_it);
        }

        process_and_publish(pipeline, image_msg, frame_info_msg);
    }

    void process_and_publish(
        Pipeline & pipeline,
        const sensor_msgs::msg::Image::SharedPtr & image_msg,
        const sentinel_interfaces::msg::FrameInfo::SharedPtr & frame_info_msg)
    {
        try {
            cv::Mat input = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
            cv::Mat processed = preprocess_image(pipeline, input);

            auto output_image = cv_bridge::CvImage(image_msg->header, "bgr8", processed).toImageMsg();
            pipeline.image_pub->publish(*output_image);

            auto output_frame_info = *frame_info_msg;
            output_frame_info.width = static_cast<uint32_t>(processed.cols);
            output_frame_info.height = static_cast<uint32_t>(processed.rows);
            pipeline.frame_info_pub->publish(output_frame_info);

            pipeline.output_images++;
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "preprocess output [%s]: %dx%d encoding=bgr8 count=%lu",
                pipeline.output_topic.c_str(),
                processed.cols,
                processed.rows,
                pipeline.output_images);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(
                get_logger(),
                "Failed preprocessing frame on %s: %s",
                pipeline.output_topic.c_str(),
                e.what()
            );
        }
    }

    cv::Mat preprocess_image(Pipeline &, const cv::Mat & input)
    {
        cv::Mat denoised;
        cv::GaussianBlur(input, denoised, cv::Size(denoise_kernel_size_, denoise_kernel_size_), 0.0);

        cv::Mat lab;
        cv::cvtColor(denoised, lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> channels;
        cv::split(lab, channels);
        auto clahe = cv::createCLAHE(clahe_clip_limit_, cv::Size(clahe_tile_size_, clahe_tile_size_));
        clahe->apply(channels[0], channels[0]);
        cv::merge(channels, lab);

        cv::Mat enhanced;
        cv::cvtColor(lab, enhanced, cv::COLOR_Lab2BGR);
        return enhanced;
    }

    // 버퍼 크기 제한 
    template<typename T>
    void trim_buffer(std::unordered_map<int64_t, T> & buffer)
    {
        while (static_cast<int>(buffer.size()) > sync_queue_size_) {
            auto oldest = std::min_element(
                buffer.begin(),
                buffer.end(),
                [](const auto & a, const auto & b) { return a.first < b.first; }
            );
            buffer.erase(oldest);
        }
    }

    static int64_t header_to_ns(const std_msgs::msg::Header & header)
    {
        return stamp_to_ns(header.stamp);
    }

    static int64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp)
    {
        return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
    }

    void log_pipeline(const char * label, const Pipeline & pipeline)
    {
        RCLCPP_INFO(get_logger(), "%s input image   : %s", label, pipeline.image_topic.c_str());
        RCLCPP_INFO(get_logger(), "%s input info    : %s", label, pipeline.frame_info_topic.c_str());
        RCLCPP_INFO(get_logger(), "%s output image  : %s", label, pipeline.output_topic.c_str());
        RCLCPP_INFO(
            get_logger(),
            "%s output info   : %s",
            label,
            pipeline.output_frame_info_topic.c_str()
        );
    }

    // 캡슐화를 위해 이 값들은 노드 내부 동작 상태라서 외부에서 접근할 필요가 없다고 판단하여 private으로 유지
    std::mutex mutex_;
    Pipeline ir_pipeline_;
    Pipeline eo_pipeline_;
    int sync_queue_size_;
    int denoise_kernel_size_;
    double clahe_clip_limit_;
    int clahe_tile_size_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePreprocessNode>());
    rclcpp::shutdown();
    return 0;
}
*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
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
        declare_parameter<std::string>("ir_image_topic", "/camera/ir");
        declare_parameter<std::string>("ir_frame_info_topic", "/camera/ir/frame_info");
        declare_parameter<std::string>("ir_output_topic", "/video/ir/preprocessed");
        declare_parameter<std::string>("ir_output_frame_info_topic", "/video/ir/preprocessed/frame_info");
        declare_parameter<std::string>("eo_image_topic", "/camera/eo");
        declare_parameter<std::string>("eo_frame_info_topic", "/camera/eo/frame_info");
        declare_parameter<std::string>("eo_output_topic", "/video/eo/preprocessed");
        declare_parameter<std::string>("eo_output_frame_info_topic", "/video/eo/preprocessed/frame_info");
        declare_parameter<int>("sync_queue_size", 30);
        declare_parameter<int>("denoise_kernel_size", 5);
        declare_parameter<std::string>("denoise_type", "median");  // "median" or "gaussian"
        declare_parameter<double>("clahe_clip_limit", 2.0);
        declare_parameter<int>("clahe_tile_size", 8);

        // Calibration: fx, fy, cx, cy, k1, k2, p1, p2, k3
        declare_parameter<double>("camera_fx", 0.0);
        declare_parameter<double>("camera_fy", 0.0);
        declare_parameter<double>("camera_cx", 0.0);
        declare_parameter<double>("camera_cy", 0.0);
        declare_parameter<std::vector<double>>("dist_coeffs", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});

        sync_queue_size_ = std::max(1, static_cast<int>(get_parameter("sync_queue_size").as_int()));
        denoise_kernel_size_ = normalize_kernel_size(static_cast<int>(get_parameter("denoise_kernel_size").as_int()));
        denoise_type_ = get_parameter("denoise_type").as_string();
        clahe_clip_limit_ = get_parameter("clahe_clip_limit").as_double();
        clahe_tile_size_ = std::max(1, static_cast<int>(get_parameter("clahe_tile_size").as_int()));

        const double fx = get_parameter("camera_fx").as_double();
        const double fy = get_parameter("camera_fy").as_double();
        const double cx = get_parameter("camera_cx").as_double();
        const double cy = get_parameter("camera_cy").as_double();
        const auto dc = get_parameter("dist_coeffs").as_double_array();

        // fx, fy, cx, cy 가 모두 0이 아니면 calibration 활성화
        calibration_enabled_ = (fx != 0.0 && fy != 0.0 && cx != 0.0 && cy != 0.0);
        if (calibration_enabled_) {
            camera_matrix_ = (cv::Mat_<double>(3, 3) <<
                fx, 0,  cx,
                0,  fy, cy,
                0,  0,  1);
            dist_coeffs_ = cv::Mat(dc).clone();
            RCLCPP_INFO(get_logger(), "Calibration enabled: fx=%.2f fy=%.2f cx=%.2f cy=%.2f", fx, fy, cx, cy);
        } else {
            RCLCPP_WARN(get_logger(), "Calibration disabled: camera_fx/fy/cx/cy not set");
        }

        ir_pipeline_ = make_pipeline(
            get_parameter("ir_image_topic").as_string(),
            get_parameter("ir_frame_info_topic").as_string(),
            get_parameter("ir_output_topic").as_string(),
            get_parameter("ir_output_frame_info_topic").as_string()
        );

        eo_pipeline_ = make_pipeline(
            get_parameter("eo_image_topic").as_string(),
            get_parameter("eo_frame_info_topic").as_string(),
            get_parameter("eo_output_topic").as_string(),
            get_parameter("eo_output_frame_info_topic").as_string()
        );

        setup_pipeline(ir_pipeline_);
        setup_pipeline(eo_pipeline_);

        RCLCPP_INFO(get_logger(), "ImagePreprocessNode started");
        log_pipeline("IR", ir_pipeline_);
        log_pipeline("EO", eo_pipeline_);
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
        std::unordered_map<int64_t, sentinel_interfaces::msg::FrameInfo::SharedPtr> frame_info_buffer;
        uint64_t input_images = 0;
        uint64_t output_images = 0;
    };

    // 타이밍 측정 결과를 담는 구조체
    struct StepTiming
    {
        double denoise_ms   = 0.0;
        double clahe_ms     = 0.0;
        double calibration_ms = 0.0;

        double total_ms() const { return denoise_ms + clahe_ms + calibration_ms; }
    };

    static int normalize_kernel_size(int size)
    {
        size = std::max(1, size);
        if (size % 2 == 0) {
            size += 1;
        }
        return size;
    }

    Pipeline make_pipeline(
        const std::string & image_topic,
        const std::string & frame_info_topic,
        const std::string & output_topic,
        const std::string & output_frame_info_topic)
    {
        Pipeline pipeline;
        pipeline.image_topic = image_topic;
        pipeline.frame_info_topic = frame_info_topic;
        pipeline.output_topic = output_topic;
        pipeline.output_frame_info_topic = output_frame_info_topic;
        return pipeline;
    }

    void setup_pipeline(Pipeline & pipeline)
    {
        pipeline.image_pub = create_publisher<sensor_msgs::msg::Image>(pipeline.output_topic, 10);
        pipeline.frame_info_pub = create_publisher<sentinel_interfaces::msg::FrameInfo>(
            pipeline.output_frame_info_topic,
            10
        );

        pipeline.image_sub = create_subscription<sensor_msgs::msg::Image>(
            pipeline.image_topic,
            rclcpp::SensorDataQoS(),
            [this, &pipeline](sensor_msgs::msg::Image::SharedPtr msg) {
                handle_image(pipeline, std::move(msg));
            }
        );

        pipeline.frame_info_sub = create_subscription<sentinel_interfaces::msg::FrameInfo>(
            pipeline.frame_info_topic,
            rclcpp::SensorDataQoS(),
            [this, &pipeline](sentinel_interfaces::msg::FrameInfo::SharedPtr msg) {
                handle_frame_info(pipeline, std::move(msg));
            }
        );
    }

    void handle_image(Pipeline & pipeline, sensor_msgs::msg::Image::SharedPtr msg)
    {
        pipeline.input_images++;
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "preprocess input [%s]: %ux%u encoding=%s count=%lu",
            pipeline.image_topic.c_str(),
            msg->width,
            msg->height,
            msg->encoding.c_str(),
            pipeline.input_images);

        const auto stamp_ns = header_to_ns(msg->header);
        if (stamp_ns < 0) {
            RCLCPP_WARN(get_logger(), "Received image without a valid stamp on %s", pipeline.image_topic.c_str());
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            pipeline.image_buffer[stamp_ns] = msg;
            trim_buffer(pipeline.image_buffer);
        }
        try_process_pair(pipeline, stamp_ns);
    }

    void handle_frame_info(Pipeline & pipeline, sentinel_interfaces::msg::FrameInfo::SharedPtr msg)
    {
        const auto stamp_ns = stamp_to_ns(msg->stamp);
        if (stamp_ns < 0) {
            RCLCPP_WARN(
                get_logger(),
                "Received FrameInfo without a valid stamp on %s",
                pipeline.frame_info_topic.c_str()
            );
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            pipeline.frame_info_buffer[stamp_ns] = msg;
            trim_buffer(pipeline.frame_info_buffer);
        }
        try_process_pair(pipeline, stamp_ns);
    }

    // 같은 타임스탬프를 가진 이미지와 프레임 정보를 찾아 처리
    void try_process_pair(Pipeline & pipeline, int64_t stamp_ns)
    {
        sensor_msgs::msg::Image::SharedPtr image_msg;
        sentinel_interfaces::msg::FrameInfo::SharedPtr frame_info_msg;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto image_it = pipeline.image_buffer.find(stamp_ns);
            auto frame_it = pipeline.frame_info_buffer.find(stamp_ns);
            if (image_it == pipeline.image_buffer.end() || frame_it == pipeline.frame_info_buffer.end()) {
                return;
            }

            image_msg = std::move(image_it->second);
            frame_info_msg = std::move(frame_it->second);
            pipeline.image_buffer.erase(image_it);
            pipeline.frame_info_buffer.erase(frame_it);
        }

        process_and_publish(pipeline, image_msg, frame_info_msg);
    }

    void process_and_publish(
        Pipeline & pipeline,
        const sensor_msgs::msg::Image::SharedPtr & image_msg,
        const sentinel_interfaces::msg::FrameInfo::SharedPtr & frame_info_msg)
    {
        try {
            cv::Mat input = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

            StepTiming timing;
            cv::Mat processed = preprocess_image(pipeline, input, timing);

            auto output_image = cv_bridge::CvImage(image_msg->header, "bgr8", processed).toImageMsg();
            pipeline.image_pub->publish(*output_image);

            auto output_frame_info = *frame_info_msg;
            output_frame_info.width = static_cast<uint32_t>(processed.cols);
            output_frame_info.height = static_cast<uint32_t>(processed.rows);
            pipeline.frame_info_pub->publish(output_frame_info);

            pipeline.output_images++;
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "preprocess output [%s]: %dx%d encoding=bgr8 count=%lu | "
                "denoise=%.2fms clahe=%.2fms calib=%.2fms total=%.2fms",
                pipeline.output_topic.c_str(),
                processed.cols,
                processed.rows,
                pipeline.output_images,
                timing.denoise_ms,
                timing.clahe_ms,
                timing.calibration_ms,
                timing.total_ms());
        } catch (const std::exception & e) {
            RCLCPP_ERROR(
                get_logger(),
                "Failed preprocessing frame on %s: %s",
                pipeline.output_topic.c_str(),
                e.what()
            );
        }
    }

    cv::Mat preprocess_image(Pipeline &, const cv::Mat & input, StepTiming & timing)
    {
        using Clock = std::chrono::steady_clock;
        using Ms    = std::chrono::duration<double, std::milli>;

        // 변경
        // 1단계: Denoise (median or gaussian)
        auto t0 = Clock::now();
        cv::Mat denoised;
        if (denoise_type_ == "gaussian") {
            cv::GaussianBlur(input, denoised,
                cv::Size(denoise_kernel_size_, denoise_kernel_size_), 0.0);
        } else {
            cv::medianBlur(input, denoised, denoise_kernel_size_);
        }
        timing.denoise_ms = Ms(Clock::now() - t0).count();

        // 2단계: CLAHE (히스토그램 평활화, L 채널에만 적용)
        t0 = Clock::now();
        cv::Mat lab;
        cv::cvtColor(denoised, lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> channels;
        cv::split(lab, channels);
        auto clahe = cv::createCLAHE(clahe_clip_limit_, cv::Size(clahe_tile_size_, clahe_tile_size_));
        clahe->apply(channels[0], channels[0]);
        cv::merge(channels, lab);
        cv::Mat enhanced;
        cv::cvtColor(lab, enhanced, cv::COLOR_Lab2BGR);
        timing.clahe_ms = Ms(Clock::now() - t0).count();

        // 3단계: 왜곡 보정 (Calibration)
        t0 = Clock::now();
        cv::Mat calibrated;
        if (calibration_enabled_) {
            cv::undistort(enhanced, calibrated, camera_matrix_, dist_coeffs_);
        } else {
            calibrated = enhanced;
        }
        timing.calibration_ms = Ms(Clock::now() - t0).count();

        return calibrated;
    }

    // 버퍼 크기 제한
    template<typename T>
    void trim_buffer(std::unordered_map<int64_t, T> & buffer)
    {
        while (static_cast<int>(buffer.size()) > sync_queue_size_) {
            auto oldest = std::min_element(
                buffer.begin(),
                buffer.end(),
                [](const auto & a, const auto & b) { return a.first < b.first; }
            );
            buffer.erase(oldest);
        }
    }

    static int64_t header_to_ns(const std_msgs::msg::Header & header)
    {
        return stamp_to_ns(header.stamp);
    }

    static int64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp)
    {
        return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
    }

    void log_pipeline(const char * label, const Pipeline & pipeline)
    {
        RCLCPP_INFO(get_logger(), "%s input image   : %s", label, pipeline.image_topic.c_str());
        RCLCPP_INFO(get_logger(), "%s input info    : %s", label, pipeline.frame_info_topic.c_str());
        RCLCPP_INFO(get_logger(), "%s output image  : %s", label, pipeline.output_topic.c_str());
        RCLCPP_INFO(
            get_logger(),
            "%s output info   : %s",
            label,
            pipeline.output_frame_info_topic.c_str()
        );
    }

    // 캡슐화를 위해 이 값들은 노드 내부 동작 상태라서 외부에서 접근할 필요가 없다고 판단하여 private으로 유지
    std::mutex mutex_;
    Pipeline ir_pipeline_;
    Pipeline eo_pipeline_;
    int sync_queue_size_;
    int denoise_kernel_size_;
    double clahe_clip_limit_;
    int clahe_tile_size_;
    std::string denoise_type_ = "median";
    bool calibration_enabled_ = false;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePreprocessNode>());
    rclcpp::shutdown();
    return 0;
}
