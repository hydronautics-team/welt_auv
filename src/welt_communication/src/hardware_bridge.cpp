#include <rclcpp/rclcpp.hpp>
#include <stingray_core_communication/hardware_bridge.h>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<HardwareBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
