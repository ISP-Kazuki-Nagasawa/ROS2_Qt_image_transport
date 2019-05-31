#include "image_receiver.h"

ImageReceiver::ImageReceiver(QObject *parent) : QObject(parent)
{
}


#ifdef ROS
void ImageReceiver::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Image message to QImage
    QImage img(msg->data.data(), msg->width, msg->height, QImage::Format_RGB888);
    img = img.rgbSwapped();

    // Send QImage
    emit OnReceiveImage(img);
}
#endif
