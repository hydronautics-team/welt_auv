#include "stingray_core_communication/udp_driver.h"
#include "welt_communication/messages/welt.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    boost::asio::io_service io_service;
    std::shared_ptr<rclcpp::Node> sender_node = rclcpp::Node::make_shared("udp_driver_sender_node");
    auto sender = UDPBridgeSender<WeltMessage>(sender_node, io_service);
    rclcpp::spin(sender_node);
    rclcpp::shutdown();
    return 0;
}
