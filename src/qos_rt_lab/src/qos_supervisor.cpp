
#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class QoSSupervisor : public rclcpp::Node {
public:
  QoSSupervisor() : Node("qos_supervisor") {
    // thresholds via diagnostics level: WARN/ERROR triggers degrade; OK for a while triggers restore
    degrade_after_warn_count_ = declare_parameter<int>("degrade_after_warn_count", 3);
    restore_after_ok_count_ = declare_parameter<int>("restore_after_ok_count", 5);

    diag_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10,
      std::bind(&QoSSupervisor::diag_cb, this, std::placeholders::_1));

    cli_degrade_ = this->create_client<std_srvs::srv::Trigger>("/qos_rt_pub/degrade");
    cli_restore_ = this->create_client<std_srvs::srv::Trigger>("/qos_rt_pub/restore");

    RCLCPP_INFO(this->get_logger(), "qos_supervisor started.");
  }

private:
  void diag_cb(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    int level = 0;
    for (auto& st : msg->status) {
      if (st.name.find("QoS Link Health") != std::string::npos) {
        level = st.level;
        break;
      }
    }
    if (level == diagnostic_msgs::msg::DiagnosticStatus::ERROR ||
        level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      warn_count_++;
      ok_count_ = 0;
      if (!degraded_ && warn_count_ >= degrade_after_warn_count_) {
        degraded_ = true; warn_count_ = 0;
        call_trigger(cli_degrade_, "degrade");
      }
    } else { // OK
      ok_count_++;
      warn_count_ = 0;
      if (degraded_ && ok_count_ >= restore_after_ok_count_) {
        degraded_ = false; ok_count_ = 0;
        call_trigger(cli_restore_, "restore");
      }
    }
  }

  void call_trigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli, const char* tag) {
    if (!cli->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Service %s not available", tag);
      return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = cli->async_send_request(req);
    // no blocking required
    RCLCPP_INFO(this->get_logger(), "Requested %s", tag);
  }

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_degrade_, cli_restore_;
  int degrade_after_warn_count_{3};
  int restore_after_ok_count_{5};
  int warn_count_{0};
  int ok_count_{0};
  bool degraded_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QoSSupervisor>());
  rclcpp::shutdown();
  return 0;
}
