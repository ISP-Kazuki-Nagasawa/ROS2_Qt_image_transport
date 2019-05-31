#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <opencv2/highgui/highgui.hpp>


/* 
 * CV array type to ROS sensor_msgs/Image type
 * https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp
 */
std::string mat_type2encoding(int mat_type)
{
    switch (mat_type) {
    case CV_8UC1:
        return "mono8";
    case CV_8UC3:
        return "bgr8";
    case CV_16SC1:
        return "mono16";
    case CV_8UC4:
        return "rgba8";
    default:
        throw std::runtime_error("Unsupported encoding type");
    }
}


/*
 * Insert CV frame to ROS sensor_msgs/Image
 * https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp
 */
void convert_frame_to_message(const cv::Mat& frame, size_t frame_id, sensor_msgs::msg::Image& msg)
{
    // copy cv information into ros message
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);
    msg.header.frame_id = std::to_string(frame_id);
}


class ImageSenderNode final : public rclcpp::Node
{
public:
    ImageSenderNode() : rclcpp::Node("image_sender")
    {
        // Publisher
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image");

        // Timer for publish data
        timer_ = this->create_wall_timer(std::chrono::milliseconds(40),
                                         std::bind(&ImageSenderNode::timerCallback, this));

        // Open camera device
        cap_.open(0);
        frame_id_ = 0;

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
            throw std::runtime_error("Could not open video stream");
        }
    }

private:
    void timerCallback()
    {
        // Publish image
        cap_ >> frame_;
        if (frame_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Capture frame is None");
            return;
        }

        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        convert_frame_to_message(frame_, frame_id_++, *msg);
        pub_->publish(msg);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    size_t frame_id_;
    cv::Mat frame_;
};


/*
 * Main
 */
int main(int argc, char* argv[])
{
    // Initialize
    rclcpp::init(argc, argv);

    // Create node and spin
    rclcpp::spin(std::make_unique<ImageSenderNode>());

    // Shutdown
    rclcpp::shutdown();
}
