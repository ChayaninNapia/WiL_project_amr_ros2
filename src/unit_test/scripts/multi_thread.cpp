#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include <chrono>
#include <thread>
#include <memory>
#include <functional>

using Trigger = example_interfaces::srv::Trigger;
using namespace std::chrono_literals;
using namespace std::placeholders;

class DemoNode : public rclcpp::Node
{
public:
  DemoNode()
  : Node("demo_node")
  {
    start_time_ = now();

    // 1) สร้าง Reentrant callback group สำหรับ Timer
    timer_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    // 2) สร้าง wall timer ระบุ callback_group
    timer_ = this->create_wall_timer(
      1s,
      [this]() {
        double elapsed = (now() - start_time_).seconds();
        RCLCPP_INFO(get_logger(),
          "[Timer] still alive at %.1f seconds", elapsed);
      },
      timer_group_);

    // 3) สร้าง Reentrant callback group สำหรับ Service
    service_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    // 4) สร้าง service ระบุ callback_group
    service_ = this->create_service<Trigger>(
      "wait_five_seconds",
      std::bind(&DemoNode::handle_service, this, _1, _2),
      rmw_qos_profile_default,
      service_group_);

    RCLCPP_INFO(get_logger(),
      "DemoNode ready. Call /wait_five_seconds to sleep 5s.");
  }

private:
  void handle_service(
    const std::shared_ptr<Trigger::Request> /*req*/,
    std::shared_ptr<Trigger::Response>      res)
  {
    RCLCPP_INFO(get_logger(), "[Service] Sleeping 5 seconds...");
    std::this_thread::sleep_for(5s);
    RCLCPP_INFO(get_logger(), "[Service] Done sleeping, sending response.");
    res->success = true;
    res->message = "Slept 5 seconds";
  }

  rclcpp::Time                          start_time_;
  rclcpp::TimerBase::SharedPtr         timer_;
  rclcpp::Service<Trigger>::SharedPtr  service_;
  rclcpp::CallbackGroup::SharedPtr     timer_group_;
  rclcpp::CallbackGroup::SharedPtr     service_group_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
