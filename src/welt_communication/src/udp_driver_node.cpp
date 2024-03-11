#include "stingray_core_communication/udp_driver.h"
#include "welt_communication/messages/welt.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    boost::asio::io_service io_service;
    std::shared_ptr<rclcpp::Node> sender_node = rclcpp::Node::make_shared("UDPDriverSender");
    auto sender = UDPBridgeSender<WeltMessage>(sender_node, io_service);
    std::shared_ptr<rclcpp::Node> receiver_node = rclcpp::Node::make_shared("UDPDriverReceiver");
    auto receiver = UDPBridgeReceiver<WeltMessage>(receiver_node, io_service);
    std::thread s([&] {
        receiver.try_receive();
        io_service.run();
        });
    executor.add_node(sender_node);
    executor.add_node(receiver_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
