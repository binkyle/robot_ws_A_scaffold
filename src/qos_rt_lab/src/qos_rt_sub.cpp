#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

class Sub : public rclcpp::Node {
public:
  Sub() : rclcpp::Node("qos_rt_sub") {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    sub_ = this->create_subscription<std_msgs::msg::Header>(
      "qos_rt_lab/header", qos,
      [this](std_msgs::msg::Header::ConstSharedPtr msg) {
        (void)msg->frame_id;  // 示例：使用字段避免未使用警告
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
