#include "cam_header.h"
#include "frame_assembler.h"
#include "udp_receiver.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <boost/asio.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sentinel_interfaces/msg/frame_info.hpp"
#include "std_msgs/msg/header.hpp"

class VideoRxNode : public rclcpp::Node
{
public:
    VideoRxNode()
    : Node("video_rx_node")
    {
        declare_parameter<std::string>("eo_device", "/dev/video0");
        declare_parameter<int>("eo_width", 1280);
        declare_parameter<int>("eo_height", 720);
        declare_parameter<int>("eo_fps", 60);
        declare_parameter<std::string>("eo_publish_topic", "/camera/eo");
        declare_parameter<std::string>("eo_frame_info_topic", "/camera/eo/frame_info");
        declare_parameter<std::string>("eo_source_name", "capturecard_eo");
        declare_parameter<std::string>("eo_frame_id", "camera_eo");

        declare_parameter<int>("ir_udp_port", 5001);
        declare_parameter<int>("ir_camera_id", static_cast<int>(CAM_ID_IR));
        declare_parameter<int>("ir_width", 640);
        declare_parameter<int>("ir_height", 480);
        declare_parameter<double>("ir_fps", 10.0);
        declare_parameter<std::string>("ir_publish_topic", "/camera/ir");
        declare_parameter<std::string>("ir_frame_info_topic", "/camera/ir/frame_info");
        declare_parameter<std::string>("ir_source_name", "jetson_udp_ir");
        declare_parameter<std::string>("ir_frame_id", "camera_ir");
        declare_parameter<int>("stale_ms", 2000);

        eo_device_ = get_parameter("eo_device").as_string();
        eo_width_ = static_cast<uint16_t>(get_parameter("eo_width").as_int());
        eo_height_ = static_cast<uint16_t>(get_parameter("eo_height").as_int());
        eo_fps_ = get_parameter("eo_fps").as_int();
        eo_source_name_ = get_parameter("eo_source_name").as_string();
        eo_ros_frame_id_ = get_parameter("eo_frame_id").as_string();

        eo_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            get_parameter("eo_publish_topic").as_string(), 10);
        eo_frame_info_pub_ = create_publisher<sentinel_interfaces::msg::FrameInfo>(
            get_parameter("eo_frame_info_topic").as_string(), 10);

        ir_pipeline_.udp_port = static_cast<uint16_t>(get_parameter("ir_udp_port").as_int());
        ir_pipeline_.camera_id = static_cast<uint8_t>(get_parameter("ir_camera_id").as_int());
        ir_pipeline_.frame_width = static_cast<uint16_t>(get_parameter("ir_width").as_int());
        ir_pipeline_.frame_height = static_cast<uint16_t>(get_parameter("ir_height").as_int());
        ir_pipeline_.expected_fps = get_parameter("ir_fps").as_double();
        ir_pipeline_.publish_topic = get_parameter("ir_publish_topic").as_string();
        ir_pipeline_.frame_info_topic = get_parameter("ir_frame_info_topic").as_string();
        ir_pipeline_.source_name = get_parameter("ir_source_name").as_string();
        ir_pipeline_.ros_frame_id = get_parameter("ir_frame_id").as_string();
        stale_ms_ = static_cast<uint32_t>(
            std::max<int64_t>(100, get_parameter("stale_ms").as_int()));

        initialize_ir_pipeline(ir_pipeline_);

        assembler_ = std::make_unique<FrameAssembler>(
            [this](AssembledFrame frame) { handle_ir_frame(std::move(frame)); },
            stale_ms_);
        io_work_ = std::make_unique<IoWorkGuard>(boost::asio::make_work_guard(io_context_));
        ir_receiver_ = std::make_unique<UdpReceiver>(
            io_context_,
            ir_pipeline_.udp_port,
            ir_pipeline_.camera_id,
            ir_pipeline_.frame_width,
            ir_pipeline_.frame_height,
            *assembler_);
        ir_receiver_->start();
        io_thread_ = std::thread([this]() { io_context_.run(); });

        eo_running_ = true;
        eo_thread_ = std::thread(&VideoRxNode::eo_capture_loop, this);

        ir_pipeline_.last_fps_time = now();
        eo_last_fps_time_ = now();
        status_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&VideoRxNode::on_status_timer, this));

        RCLCPP_INFO(get_logger(), "VideoRxNode started");
        RCLCPP_INFO(
            get_logger(),
            "EO device : %s %ux%u @ %dfps",
            eo_device_.c_str(),
            eo_width_,
            eo_height_,
            eo_fps_);
        RCLCPP_INFO(
            get_logger(),
            "IR UDP    : port=%u %ux%u @ %.1ffps",
            ir_pipeline_.udp_port,
            ir_pipeline_.frame_width,
            ir_pipeline_.frame_height,
            ir_pipeline_.expected_fps);
    }

    ~VideoRxNode() override
    {
        eo_running_ = false;
        if (eo_thread_.joinable()) {
            eo_thread_.join();
        }

        if (ir_receiver_) {
            ir_receiver_->stop();
        }
        io_work_.reset();
        io_context_.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
    }

private:
    struct IrPipeline
    {
        uint16_t udp_port{};
        uint8_t camera_id{};
        uint16_t frame_width{};
        uint16_t frame_height{};
        double expected_fps{0.0};
        std::string publish_topic;
        std::string frame_info_topic;
        std::string source_name;
        std::string ros_frame_id;
        std::atomic<uint32_t> published_frames{0};
        std::atomic<double> current_fps{0.0};
        uint32_t fps_counter{0};
        rclcpp::Time last_fps_time{0, 0, RCL_SYSTEM_TIME};
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
        rclcpp::Publisher<sentinel_interfaces::msg::FrameInfo>::SharedPtr frame_info_pub;
    };
3
    void eo_capture_loop()
    {
        const std::string pipeline =
            "v4l2src device=" + eo_device_ + " io-mode=2 do-timestamp=true ! "
            "image/jpeg,width=" + std::to_string(eo_width_) +
            ",height=" + std::to_string(eo_height_) +
            ",framerate=" + std::to_string(eo_fps_) + "/1 ! "
            "jpegdec ! videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false";

        RCLCPP_INFO(get_logger(), "EO: opening GStreamer pipeline: %s", pipeline.c_str());

        cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
        RCLCPP_INFO(get_logger(), "EO: VideoCapture open returned");
        if (!cap.isOpened()) {
            RCLCPP_ERROR(
                get_logger(),
                "EO: failed to open GStreamer pipeline: %s",
                pipeline.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "EO: capture started: %s", pipeline.c_str());

        cv::Mat frame;
        while (eo_running_) {
            if (!cap.read(frame) || frame.empty()) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "EO: failed to read frame");
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            const rclcpp::Time stamp = now();

            std_msgs::msg::Header header;
            header.stamp = stamp;
            header.frame_id = eo_ros_frame_id_;

            auto image_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            eo_image_pub_->publish(*image_msg);

            update_eo_fps();
            sentinel_interfaces::msg::FrameInfo info_msg;
            info_msg.stamp = stamp;
            info_msg.width = static_cast<uint32_t>(frame.cols);
            info_msg.height = static_cast<uint32_t>(frame.rows);
            info_msg.fps = static_cast<float>(eo_current_fps_.load());
            info_msg.source = eo_source_name_;
            eo_frame_info_pub_->publish(info_msg);

            eo_published_frames_.fetch_add(1);
        }
        cap.release();
        RCLCPP_INFO(get_logger(), "EO: capture loop stopped");
    }

    void update_eo_fps()
    {
        eo_fps_counter_++;
        const auto now_time = now();
        const double elapsed = (now_time - eo_last_fps_time_).seconds();
        if (elapsed >= 1.0) {
            eo_current_fps_.store(static_cast<double>(eo_fps_counter_) / elapsed);
            eo_fps_counter_ = 0;
            eo_last_fps_time_ = now_time;
        }
    }

    void handle_ir_frame(AssembledFrame frame)
    {
        const size_t expected_yuyv_bytes =
            static_cast<size_t>(frame.width) * static_cast<size_t>(frame.height) * 2U;
        if (frame.width == 0 || frame.height == 0 ||
            frame.data.size() != expected_yuyv_bytes)
        {
            RCLCPP_WARN(
                get_logger(),
                "Invalid YUYV frame size: expected=%zu actual=%zu",
                expected_yuyv_bytes,
                frame.data.size());
            return;
        }

        cv::Mat yuyv(
            static_cast<int>(frame.height),
            static_cast<int>(frame.width),
            CV_8UC2,
            frame.data.data());
        cv::Mat image;
        cv::cvtColor(yuyv, image, cv::COLOR_YUV2BGR_YUYV);
        if (image.empty()) {
            RCLCPP_WARN(get_logger(), "Failed to convert YUYV frame");
            return;
        }

        const int64_t stamp_ns = static_cast<int64_t>(frame.timestamp_ms) * 1000000LL;
        const rclcpp::Time stamp(stamp_ns, RCL_SYSTEM_TIME);

        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = ir_pipeline_.ros_frame_id;

        auto image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
        ir_pipeline_.image_pub->publish(*image_msg);

        sentinel_interfaces::msg::FrameInfo info_msg;
        info_msg.stamp = stamp;
        info_msg.frame_id = frame.frame_id;
        info_msg.width = frame.width;
        info_msg.height = frame.height;
        info_msg.fps = static_cast<float>(
            ir_pipeline_.current_fps.load() > 0.0
                ? ir_pipeline_.current_fps.load()
                : ir_pipeline_.expected_fps);
        info_msg.source = ir_pipeline_.source_name;
        ir_pipeline_.frame_info_pub->publish(info_msg);

        ir_pipeline_.published_frames.fetch_add(1);
        update_ir_fps();
    }

    void update_ir_fps()
    {
        ir_pipeline_.fps_counter++;
        const auto now_time = now();
        const double elapsed = (now_time - ir_pipeline_.last_fps_time).seconds();
        if (elapsed >= 1.0) {
            ir_pipeline_.current_fps.store(
                static_cast<double>(ir_pipeline_.fps_counter) / elapsed);
            ir_pipeline_.fps_counter = 0;
            ir_pipeline_.last_fps_time = now_time;
        }
    }

    void initialize_ir_pipeline(IrPipeline & p)
    {
        p.image_pub = create_publisher<sensor_msgs::msg::Image>(p.publish_topic, 10);
        p.frame_info_pub = create_publisher<sentinel_interfaces::msg::FrameInfo>(
            p.frame_info_topic, 10);
    }

    void on_status_timer()
    {
        if (assembler_) {
            assembler_->purge_stale();
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "EO fps=%.1f frames=%u | IR fps=%.1f frames=%u",
            eo_current_fps_.load(),
            eo_published_frames_.load(),
            ir_pipeline_.current_fps.load(),
            ir_pipeline_.published_frames.load());
    }

    std::string eo_device_;
    uint16_t eo_width_{};
    uint16_t eo_height_{};
    int eo_fps_{};
    std::string eo_source_name_;
    std::string eo_ros_frame_id_;
    std::atomic<bool> eo_running_{false};
    std::thread eo_thread_;
    std::atomic<uint32_t> eo_published_frames_{0};
    std::atomic<double> eo_current_fps_{0.0};
    uint32_t eo_fps_counter_{0};
    rclcpp::Time eo_last_fps_time_{0, 0, RCL_SYSTEM_TIME};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr eo_image_pub_;
    rclcpp::Publisher<sentinel_interfaces::msg::FrameInfo>::SharedPtr eo_frame_info_pub_;

    IrPipeline ir_pipeline_;
    std::unique_ptr<FrameAssembler> assembler_;
    std::unique_ptr<UdpReceiver> ir_receiver_;
    using IoWorkGuard = boost::asio::executor_work_guard<
        boost::asio::io_context::executor_type>;
    boost::asio::io_context io_context_;
    std::unique_ptr<IoWorkGuard> io_work_;
    std::thread io_thread_;
    uint32_t stale_ms_{2000};

    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoRxNode>());
    rclcpp::shutdown();
    return 0;
}
