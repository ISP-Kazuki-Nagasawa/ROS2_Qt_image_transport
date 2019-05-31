#include <QGuiApplication>
#include <QQmlApplicationEngine>

#include "video_provider.h"
#include "image_receiver.h"

#ifdef ROS
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

#include <iostream>

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    // Connect to VideoCapture
    VideoProvider* videoProvider = new VideoProvider();
    engine.addImageProvider(QLatin1String("VideoCapture"), videoProvider);

    // Image Receiver
    ImageReceiver imageReceiver;

    // Connect ImageReceiver to VideoProvider
    QObject::connect(&imageReceiver, SIGNAL(OnReceiveImage(QImage)), videoProvider, SLOT(OnReceiveImage(QImage)));

    // Load QML
    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
    if (engine.rootObjects().isEmpty())
        return -1;

#ifdef ROS
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Create subscriber node
    auto node = rclcpp::Node::make_shared("qt_subscriber");

    // Subscription
    auto cb_group = node->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    auto sub = node->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image",
                std::bind(&ImageReceiver::imageCallback, &imageReceiver, std::placeholders::_1),
                rmw_qos_profile_sensor_data,
                cb_group);

    // ROS and application loop
    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok()) {
        app.processEvents();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
#else
    return app.exec();
#endif
}
