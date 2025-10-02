#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class QoSPublisher : public rclcpp::Node {
public:
  QoSPublisher()
  : Node("qos_rt_pub")
  {
    // 默认参数
    declare_parameter<int>("depth", 10);
    declare_parameter<double>("rate_hz", 10.0);

    // 读取参数
    depth_   = get_parameter("depth").as_int();
    rate_hz_ = get_parameter("rate_hz").as_double();
    pristine_depth_ = depth_;
    pristine_rate_  = rate_hz_;

    create_pub();
    create_timer();

    // Reset 服务：Trigger::Request 是空的；Response 有 success/message
    reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        depth_   = pristine_depth_;
        rate_hz_ = pristine_rate_;
        create_pub();
        create_timer();
        res->success = true;
        res->message = "Restored";
        RCLCPP_INFO(this->get_logger(), "Reset to depth=%d, rate=%.3f Hz", depth_, rate_hz_);
      }
    );

    RCLCPP_INFO(get_logger(), "qos_rt_pub started: depth=%d, rate=%.3f Hz", depth_, rate_hz_);
  }

private:
  void create_pub() {
    rclcpp::QoS qos(rclcpp::KeepLast(depth_));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    pub_ = this->create_publisher<std_msgs::msg::Header>("qos_rt_lab/header", qos);
  }

  void create_timer() {
    if (timer_) timer_.reset();
    auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() {
        std_msgs::msg::Header msg;
        msg.stamp = this->now();
        msg.frame_id = "map";
        pub_->publish(msg);
      }
    );
  }

  // 成员
  int depth_{10};
  double rate_hz_{10.0};
  int pristine_depth_{10};
  double pristine_rate_{10.0};

  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QoSPublisher>());
  rclcpp::shutdown();
  return 0;
}
