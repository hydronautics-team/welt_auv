#include <rclcpp/rclcpp.hpp>
#include "stingray_core_communication/udp_driver.h"
#include "welt_communication/messages/welt.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    boost::asio::io_service io_service;
    auto sender = std::make_shared<UDPBridgeSender>(io_service);
    auto receiver = std::make_shared<UDPBridgeReceiver>(io_service);
    std::thread s([&] {
        receiver->try_receive();
        io_service.run();
        });
    executor.add_node(sender);
    executor.add_node(receiver);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
