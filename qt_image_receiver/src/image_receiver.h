#ifndef IMAGE_RECEIVER_H
#define IMAGE_RECEIVER_H

#include <QObject>
#include <QImage>

#ifdef ROS
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif


class ImageReceiver : public QObject
{
    Q_OBJECT
public:
    explicit ImageReceiver(QObject *parent = nullptr);

#ifdef ROS
public:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
#endif

signals:
    void OnReceiveImage(QImage image);
};

#endif // IMAGE_RECEIVER_H
