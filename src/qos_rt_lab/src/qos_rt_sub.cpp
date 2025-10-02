#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

class Sub : public rclcpp::Node {
public:
  Sub() : rclcpp::Node("qos_rt_sub") {
    // 同样用花括号，避免被当成函数声明
    rclcpp::QoS qos{ rclcpp::KeepLast(static_cast<size_t>(10)) };
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    sub_ = this->create_subscription<std_msgs::msg::Header>(
      "qos_rt_lab/header", qos,
      [this](std_msgs::msg::Header::ConstSharedPtr msg) {
        (void)msg->frame_id;
        RCLCPP_INFO(get_logger(), "recv header stamp=%.3f",
                    rclcpp::Time(msg->stamp).seconds());
      }
    );
  }

private:
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sub>());
  rclcpp::shutdown();
  return 0;
}
