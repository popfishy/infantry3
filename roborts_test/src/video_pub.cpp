#include <chrono>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class VideoPub : public rclcpp::Node
{
public:
    VideoPub() : Node("video_pub")
    {    
        RCLCPP_INFO(this->get_logger(), "Start video stream !");
        video_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video", 5);
        //video_compressed_publisher_ =
            //this->create_publisher<sensor_msgs::msg::CompressedImage>("video/compressed", 5);
        cap.open("/home/yjq/sentry1.0/src/roborts_video/MV/live.avi");
        if (!cap.isOpened())
        {
            RCLCPP_INFO(this->get_logger(), "Read video Failed !");
            return;
        }
        timer_ = this->create_wall_timer(50ms,std::bind(&VideoPub::timer_callback,this));
    }

private:
    void timer_callback()
    {
        while (rclcpp::ok())
        {
            cap >> image;
            cv::waitKey(50);
            img_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
            //img_compressed_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toCompressedImageMsg();
            video_publisher_->publish(*img_);
        }
    }


private:
    cv::VideoCapture cap;
    cv::Mat image;
    // ROS2
    sensor_msgs::msg::Image::SharedPtr img_;
    sensor_msgs::msg::CompressedImage::SharedPtr img_compressed_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr video_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr video_compressed_publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPub>());
    rclcpp::shutdown();
    return 0;
}
