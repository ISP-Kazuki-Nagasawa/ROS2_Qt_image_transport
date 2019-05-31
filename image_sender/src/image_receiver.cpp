#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


/*
 * ROS sensor_msgs/Image to CY array type
 * https://github.com/ros2/demos/blob/master/image_tools/src/showimage.cpp
 */
int encoding2mat_type(const std::string & encoding)
{
    if (encoding == "mono8") {
        return CV_8UC1;
    } else if (encoding == "bgr8") {
        return CV_8UC3;
    } else if (encoding == "mono16") {
        return CV_16SC1;
    } else if (encoding == "rgba8") {
        return CV_8UC4;
    } else if (encoding == "bgra8") {
        return CV_8UC4;
    } else if (encoding == "32FC1") {
        return CV_32FC1;
    } else if (encoding == "rgb8") {
        return CV_8UC3;
    } else {
        throw std::runtime_error("Unsupported encoding type");
    }
}


class ImageReceiverNode final : public rclcpp::Node
{
public:
    ImageReceiverNode() : rclcpp::Node("image_receiver")
    {
        // Subscription
        sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image",
                                                                  std::bind(&ImageReceiverNode::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding),
                      const_cast<unsigned char *>(msg->data.data()), msg->step);

        if (msg->encoding == "rgb8") {
            cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
        }

        cv::Mat display_frame = frame;
        cv::imshow("OpenCV image receiver", display_frame);
        cv::waitKey(1);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};


/*
 * Main
 */
int main(int argc, char* argv[])
{
    // Initialize
    rclcpp::init(argc, argv);

    // Create node and spin
    rclcpp::spin(std::make_unique<ImageReceiverNode>());

    // Shutdown
    rclcpp::shutdown();
}
