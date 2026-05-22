#include "cam_header.h"
#include "frame_assembler.h"
#include "udp_receiver.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/asio.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sentinel_interfaces/msg/frame_info.hpp"
#include "std_msgs/msg/header.hpp"

// 완성된 프레임을 받아서, JPEG 바이트를 이미지로 decode
class VideoRxNode : public rclcpp::Node
{
public:
    VideoRxNode()
    : Node("video_rx_node")
    {
        declare_parameter<int>("ir_udp_port", 5001);
        declare_parameter<int>("eo_udp_port", 5000);
        declare_parameter<int>("ir_camera_id", static_cast<int>(CAM_ID_IR));
        declare_parameter<int>("eo_camera_id", static_cast<int>(CAM_ID_EO));
        declare_parameter<std::string>("ir_publish_topic", "/camera/ir");
        declare_parameter<std::string>("ir_frame_info_topic", "/camera/ir/frame_info");
        declare_parameter<std::string>("ir_source_name", "jetson_udp_ir");
        declare_parameter<std::string>("ir_frame_id", "camera_ir");
        declare_parameter<double>("ir_fps", 10.0);
        declare_parameter<int>("ir_width", 640);
        declare_parameter<int>("ir_height", 480);
        declare_parameter<std::string>("eo_publish_topic", "/camera/eo");
        declare_parameter<std::string>("eo_frame_info_topic", "/camera/eo/frame_info");
        declare_parameter<std::string>("eo_source_name", "jetson_udp_eo");
        declare_parameter<std::string>("eo_frame_id", "camera_eo");
        declare_parameter<double>("eo_fps", 10.0);
        declare_parameter<int>("eo_width", 1280);
        declare_parameter<int>("eo_height", 720);
        declare_parameter<int>("stale_ms", 2000);

        ir_pipeline_.udp_port = static_cast<uint16_t>(get_parameter("ir_udp_port").as_int());
        ir_pipeline_.camera_id = static_cast<uint8_t>(get_parameter("ir_camera_id").as_int());
        ir_pipeline_.publish_topic = get_parameter("ir_publish_topic").as_string();
        ir_pipeline_.frame_info_topic = get_parameter("ir_frame_info_topic").as_string();
        ir_pipeline_.source_name = get_parameter("ir_source_name").as_string();
        ir_pipeline_.ros_frame_id = get_parameter("ir_frame_id").as_string();
        ir_pipeline_.expected_fps = get_parameter("ir_fps").as_double();
        ir_pipeline_.frame_width = static_cast<uint16_t>(get_parameter("ir_width").as_int());
        ir_pipeline_.frame_height = static_cast<uint16_t>(get_parameter("ir_height").as_int());

        eo_pipeline_.udp_port = static_cast<uint16_t>(get_parameter("eo_udp_port").as_int());
        eo_pipeline_.camera_id = static_cast<uint8_t>(get_parameter("eo_camera_id").as_int());
        eo_pipeline_.publish_topic = get_parameter("eo_publish_topic").as_string();
        eo_pipeline_.frame_info_topic = get_parameter("eo_frame_info_topic").as_string();
        eo_pipeline_.source_name = get_parameter("eo_source_name").as_string();
        eo_pipeline_.ros_frame_id = get_parameter("eo_frame_id").as_string();
        eo_pipeline_.expected_fps = get_parameter("eo_fps").as_double();
        eo_pipeline_.frame_width = static_cast<uint16_t>(get_parameter("eo_width").as_int());
        eo_pipeline_.frame_height = static_cast<uint16_t>(get_parameter("eo_height").as_int());

        stale_ms_ = static_cast<uint32_t>(std::max<int64_t>(100, get_parameter("stale_ms").as_int()));
       
        // 파라미터 값을 각 CameraPipeline 구조체에 채워넣고, FrameAssembler 객체를 만들 때 자신을 콜백으로 넘겨줌
        assembler_ = std::make_unique<FrameAssembler>(
            [this](AssembledFrame frame) { this->handle_frame(std::move(frame)); },
            stale_ms_);

        initialize_pipeline(ir_pipeline_);
        initialize_pipeline(eo_pipeline_);

        io_work_ = std::make_unique<IoWorkGuard>(boost::asio::make_work_guard(io_context_));

        ir_receiver_ = std::make_unique<UdpReceiver>(
            io_context_,
            ir_pipeline_.udp_port,
            ir_pipeline_.camera_id,
            ir_pipeline_.frame_width,
            ir_pipeline_.frame_height,
            *assembler_);
        eo_receiver_ = std::make_unique<UdpReceiver>(
            io_context_,
            eo_pipeline_.udp_port,
            eo_pipeline_.camera_id,
            eo_pipeline_.frame_width,
            eo_pipeline_.frame_height,
            *assembler_);
        ir_receiver_->start();
        eo_receiver_->start();
        io_thread_ = std::thread([this]() { io_context_.run(); });

        ir_pipeline_.last_fps_time = now();
        eo_pipeline_.last_fps_time = now();
        status_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&VideoRxNode::on_status_timer, this));

        RCLCPP_INFO(get_logger(), "VideoRxNode started");
        log_pipeline("IR", ir_pipeline_);
        log_pipeline("EO", eo_pipeline_);
    }

    ~VideoRxNode() override
    {
        if (ir_receiver_) {
            ir_receiver_->stop();
        }
        if (eo_receiver_) {
            eo_receiver_->stop();
        }
        io_work_.reset();
        io_context_.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
    }

private:
    // 카메라 1개(IR 또는 EO)에 필요한 상태를 한 번에 묶은 자료구조
    struct CameraPipeline
    {
        uint16_t udp_port{};
        uint8_t camera_id{};
        std::string publish_topic;
        std::string frame_info_topic;
        std::string source_name;
        std::string ros_frame_id;
        double expected_fps{0.0};
        uint16_t frame_width{0};
        uint16_t frame_height{0};
        std::atomic<uint32_t> published_frames{0};
        std::atomic<double> current_fps{0.0};
        uint32_t fps_counter{0};
        rclcpp::Time last_fps_time{0, 0, RCL_SYSTEM_TIME};
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
        rclcpp::Publisher<sentinel_interfaces::msg::FrameInfo>::SharedPtr frame_info_pub;
    };

    void handle_frame(AssembledFrame frame)
    {
        // 카메라 분기 
        CameraPipeline * pipeline = get_pipeline(frame.camera_id);
        if (pipeline == nullptr) {
            return;
        }

        const size_t expected_yuyv_bytes =
            static_cast<size_t>(frame.width) * static_cast<size_t>(frame.height) * 2U;
        if (frame.width == 0 || frame.height == 0 || frame.data.size() != expected_yuyv_bytes) {
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
            RCLCPP_WARN(get_logger(), "Failed to convert assembled YUYV frame");
            return;
        }

        const int64_t stamp_ns = static_cast<int64_t>(frame.timestamp_ms) * 1000000LL;
        const rclcpp::Time stamp(stamp_ns, RCL_SYSTEM_TIME);

        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = pipeline->ros_frame_id;

        auto image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
        pipeline->image_pub->publish(*image_msg);

        sentinel_interfaces::msg::FrameInfo frame_info_msg;
        frame_info_msg.stamp = stamp;
        frame_info_msg.frame_id = frame.frame_id;
        frame_info_msg.width = frame.width;
        frame_info_msg.height = frame.height;
        frame_info_msg.fps = static_cast<float>(
            pipeline->current_fps.load() > 0.0 ? pipeline->current_fps.load() : pipeline->expected_fps);
        frame_info_msg.source = pipeline->source_name;
        pipeline->frame_info_pub->publish(frame_info_msg);

        pipeline->published_frames.fetch_add(1);
        update_fps(*pipeline);
    }

    void update_fps(CameraPipeline & pipeline)
    {
        pipeline.fps_counter++;
        const auto now_time = now();
        const double elapsed = (now_time - pipeline.last_fps_time).seconds();
        if (elapsed >= 1.0) {
            pipeline.current_fps.store(static_cast<double>(pipeline.fps_counter) / elapsed);
            pipeline.fps_counter = 0;
            pipeline.last_fps_time = now_time;
        }
    }

    void on_status_timer()
    {
        assembler_->purge_stale();
        const auto stats = assembler_->stats();
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Receiver running | packets=%zu frames=%zu dropped=%zu",
            stats.packets_received,
            stats.frames_completed,
            stats.frames_dropped);
    }

    void initialize_pipeline(CameraPipeline & pipeline)
    {
        pipeline.image_pub = create_publisher<sensor_msgs::msg::Image>(pipeline.publish_topic, 10);
        pipeline.frame_info_pub =
            create_publisher<sentinel_interfaces::msg::FrameInfo>(pipeline.frame_info_topic, 10);
    }

    CameraPipeline * get_pipeline(uint8_t camera_id)
    {
        if (camera_id == ir_pipeline_.camera_id) {
            return &ir_pipeline_;
        }
        if (camera_id == eo_pipeline_.camera_id) {
            return &eo_pipeline_;
        }
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Received frame for unknown camera_id=0x%02X",
            camera_id);
        return nullptr;
    }

    void log_pipeline(const char * label, const CameraPipeline & pipeline)
    {
        RCLCPP_INFO(get_logger(), "%s UDP port      : %u", label, pipeline.udp_port);
        RCLCPP_INFO(get_logger(), "%s camera id     : 0x%02X", label, pipeline.camera_id);
        RCLCPP_INFO(get_logger(), "%s image topic   : %s", label, pipeline.publish_topic.c_str());
        RCLCPP_INFO(get_logger(), "%s info topic    : %s", label, pipeline.frame_info_topic.c_str());
    }

    std::unique_ptr<FrameAssembler> assembler_;
    std::unique_ptr<UdpReceiver> ir_receiver_;
    std::unique_ptr<UdpReceiver> eo_receiver_;
    using IoWorkGuard = boost::asio::executor_work_guard<boost::asio::io_context::executor_type>;
    boost::asio::io_context io_context_;
    std::unique_ptr<IoWorkGuard> io_work_;
    std::thread io_thread_;
    CameraPipeline ir_pipeline_;
    CameraPipeline eo_pipeline_;
    uint32_t stale_ms_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoRxNode>());
    rclcpp::shutdown();
    return 0;
}
