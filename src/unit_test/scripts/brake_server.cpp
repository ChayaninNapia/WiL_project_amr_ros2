#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include "amr_interfaces/srv/brake_control.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using BrakeControl = amr_interfaces::srv::BrakeControl;

class BrakeServiceNode : public rclcpp::Node
{
public:
  BrakeServiceNode()
  : Node("brake_service_cpp")
  {
    // 1) Declare parameters with defaults
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baud", 115200);

    // 2) Get their values
    this->get_parameter("port", port_);
    this->get_parameter("baud", baud_);

    // 3) Configure and open serial port
    controller_.setPort(port_);
    controller_.setBaudrate(static_cast<uint32_t>(baud_));
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    controller_.setTimeout(to);
    try {
      controller_.open();
    } catch (const serial::IOException &e) {
      RCLCPP_FATAL(get_logger(),
        "Unable to open serial port '%s': %s",
        port_.c_str(), e.what());
      rclcpp::shutdown();
      return;
    }
    if (!controller_.isOpen()) {
      RCLCPP_FATAL(get_logger(),
        "Serial port '%s' not open", port_.c_str());
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(),
      "Serial port '%s' opened at %d baud",
      port_.c_str(), baud_);

    // 4) Create the service
    service_ = create_service<BrakeControl>(
      "/amr_service/brake_control",
      std::bind(&BrakeServiceNode::handle_brake_request, this, _1, _2)
    );
    RCLCPP_INFO(get_logger(), "Brake service is ready.");
  }

private:
  void handle_brake_request(
    const std::shared_ptr<BrakeControl::Request> request,
    std::shared_ptr<BrakeControl::Response>      response)
  {
    if (request->enable) {
      RCLCPP_INFO(get_logger(), "Brake ENABLED");
      controller_.write("!DS 0\r");
      response->message = "Brake engaged";
    } else {
      RCLCPP_INFO(get_logger(), "Brake DISABLED");
      controller_.write("!DS 03\r");
      response->message = "Brake released";
    }
    response->success = true;
  }

  rclcpp::Service<BrakeControl>::SharedPtr service_;
  serial::Serial                          controller_;
  std::string                             port_;
  int                                     baud_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrakeServiceNode>());
  rclcpp::shutdown();
  return 0;
}
