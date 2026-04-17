#include <chrono> // 타이머 주기 설정을 위해
#include <cstdint> // 고정 크기 정수 타입
#include <memory>  // shared_ptr 같은 스마트 포인터 사용 (메모리 자동 관리)
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp" // OpenCV와 ROS 메시지 간 변환을 위해
#include "opencv2/opencv.hpp" // OpenCV 라이브러리

#include "sentinel_interfaces/msg/frame_info.hpp" // 사용자 정의 메시지 타입 
#include "sentinel_interfaces/msg/video_rx_status.hpp"

class VideoRxNode : public rclcpp::Node // 로스 노드 클래스 정의
{
public:
    VideoRxNode()
    : Node("video_rx_node"), // 노드 이름을 "video_rx_node"로 설정
      fps_(10.0),            // 멤버 변수 초기화
      frame_count_(0) 
    {
        // ROS2 파라미터 선언 및 초기화
        // 노드의 설정을 외부에서 바꿀 수 있도록 함
        this->declare_parameter<std::string>("video_path", "");
        this->declare_parameter<std::string>("publish_topic", "/video/raw");
        this->declare_parameter<double>("fps", 10.0);
        
        // 파라미터 값을 읽어와 멤버 변수에 저장
        video_path_ = this->get_parameter("video_path").as_string();
        publish_topic_ = this->get_parameter("publish_topic").as_string();
        fps_ = this->get_parameter("fps").as_double();

        // 비디오 경로가 비어있는지 확인
        if (video_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'video_path' is empty.");
            throw std::runtime_error("video_path is empty");
        }
        // fps 값이 유효한지 확인
        if (fps_ <= 0.0) {
            // 에러 로그를 출력하고 기본값으로 설정
            RCLCPP_WARN(this->get_logger(), "Invalid fps: %.2f. Fallback to 10.0", fps_);
            fps_ = 10.0;
        }
        // 영상 파일이 정상적으로 열렸는지 확인 
        cap_.open(video_path_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video: %s", video_path_.c_str());
            throw std::runtime_error("Failed to open video");
        }

        // 토픽에 메시지를 보내기 위한 퍼블리셔(발행키) 생성
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(publish_topic_, 10);
        frame_info_pub_ =
            this->create_publisher<sentinel_interfaces::msg::FrameInfo>("/video/frame_info", 10);
        status_pub_ =
            this->create_publisher<sentinel_interfaces::msg::VideoRxStatus>("/video/rx_status", 10);

        // 프레임 발행 주기을 초 단위 시간으로 계산 
        auto period = std::chrono::duration<double>(1.0 / fps_);
        // 계산한 시간을 ms로 변환 
        auto period_ms = std::chrono::duration_cast<std::chrono::milliseconds>(period);

        // period_ms가 0 이하인 경우 기본값으로 설정
        if (period_ms.count() <= 0) {
            period_ms = std::chrono::milliseconds(100);
        }

        // 정해진 주기마다 timer_callback 함수를 호출하는 타이머 생성
        timer_ = this->create_wall_timer(
            period_ms,
            std::bind(&VideoRxNode::timer_callback, this));

        // 어떤 노드가 어떤 설정으로 시작했는지 로그로 출력
        RCLCPP_INFO(this->get_logger(), "VideoRxNode started.");
        RCLCPP_INFO(this->get_logger(), "Video path: %s", video_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish topic: %s", publish_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "FPS: %.2f", fps_);
    }

private:
    // 노드 상태를 다른 노드에세 알려주는 함수 
    // /video/rx_status 토픽을 담당 
    void publish_status(bool is_ok, const std::string & message)
    {
        sentinel_interfaces::msg::VideoRxStatus status_msg;
        status_msg.stamp = this->now();
        status_msg.is_ok = is_ok;
        status_msg.message = message;
        status_msg.video_path = video_path_;
        status_msg.published_frames = frame_count_;
        status_pub_->publish(status_msg);
    }

    // 타이머가 호출하는 콜백 함수
    void timer_callback()
    {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            publish_status(false, "End of video reached");
            RCLCPP_INFO(this->get_logger(), "End of video reached.");
            rclcpp::shutdown();
            return;
        }

        const auto now = this->now();

        std_msgs::msg::Header header;
        header.stamp = now;
        header.frame_id = "camera_frame";

        auto image_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        image_pub_->publish(*image_msg);

        sentinel_interfaces::msg::FrameInfo frame_info_msg;
        frame_info_msg.stamp = now;
        frame_info_msg.frame_id = frame_count_;
        frame_info_msg.width = static_cast<uint32_t>(frame.cols);
        frame_info_msg.height = static_cast<uint32_t>(frame.rows);
        frame_info_msg.fps = static_cast<float>(fps_);
        frame_info_msg.source = video_path_;
        frame_info_pub_->publish(frame_info_msg);

        publish_status(true, "Frame published successfully");

        frame_count_++;
    }

    // 멤버 변수 선언
    std::string video_path_;
    std::string publish_topic_;
    double fps_;
    uint32_t frame_count_;

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sentinel_interfaces::msg::FrameInfo>::SharedPtr frame_info_pub_;
    rclcpp::Publisher<sentinel_interfaces::msg::VideoRxStatus>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoRxNode>());
    rclcpp::shutdown();
    return 0;
}