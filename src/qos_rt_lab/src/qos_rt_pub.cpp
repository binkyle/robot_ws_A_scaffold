
#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class QoSPublisher : public rclcpp::Node {
public:
  QoSPublisher() : Node("qos_rt_pub") {
    // Declare parameters
    reliability_ = declare_parameter<std::string>("reliability", "reliable"); // reliable | best_effort
    history_ = declare_parameter<std::string>("history", "keep_last"); // keep_last | keep_all
    depth_ = declare_parameter<int>("depth", 10);
    deadline_ms_ = declare_parameter<int>("deadline_ms", 0);
    latency_budget_ms_ = declare_parameter<int>("latency_budget_ms", 0);
    rate_hz_ = declare_parameter<int>("rate_hz", 1000);

    fault_drop_n_ = declare_parameter<int>("fault_drop_n", 0);         // drop every Nth msg (0=off)
    fault_delay_us_ = declare_parameter<int>("fault_delay_us", 0);     // extra delay before publish

    // Save pristine config for restore
    pristine_ = {reliability_, history_, depth_, deadline_ms_, latency_budget_ms_, rate_hz_};

    // Create publisher & timer
    recreate_pub();
    create_timer();

    // Services: degrade / restore
    degrade_srv_ = create_service<std_srvs::srv::Trigger>("degrade",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res){
        RCLCPP_WARN(this->get_logger(), "Degrade requested: best_effort + lower rate");
        reliability_ = "best_effort";
        history_ = "keep_last";
        depth_ = std::min(depth_, 5);
        rate_hz_ = std::min(rate_hz_, 200);
        recreate_pub();
        create_timer();
        res->success = true; res->message = "Degraded";
      });

    restore_srv_ = create_service<std_srvs::srv::Trigger>("restore",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res){
        RCLCPP_INFO(this->get_logger(), "Restore requested: reverting to pristine QoS");
        reliability_ = pristine_.reliability;
        history_ = pristine_.history;
        depth_ = pristine_.depth;
        deadline_ms_ = pristine_.deadline_ms;
        latency_budget_ms_ = pristine_.latency_budget_ms;
        rate_hz_ = pristine_.rate_hz;
        recreate_pub();
        create_timer();
        res->success = true; res->message = "Restored";
      });
  }

private:
  struct Pristine {
    std::string reliability;
    std::string history;
    int depth;
    int deadline_ms;
    int latency_budget_ms;
    int rate_hz;
  } pristine_;

  void recreate_pub() {
    rclcpp::QoS qos = (history_ == "keep_all") ? rclcpp::QoS(rclcpp::KeepAll())
                                               : rclcpp::QoS(rclcpp::KeepLast(depth_));
    if (reliability_ == "best_effort") qos.best_effort(); else qos.reliable();
    if (deadline_ms_ > 0) qos.deadline(std::chrono::milliseconds(deadline_ms_));
    if (latency_budget_ms_ > 0) qos.latency_budget(std::chrono::milliseconds(latency_budget_ms_));

    pub_ = this->create_publisher<std_msgs::msg::Header>("qos_rt/tick", qos);
    RCLCPP_INFO(this->get_logger(), "Publisher recreated: rel=%s hist=%s depth=%d rate=%dHz",
                reliability_.c_str(), history_.c_str(), depth_, rate_hz_);
  }

  void create_timer() {
    if (timer_) timer_->cancel();
    auto period = std::chrono::microseconds(1000000 / std::max(1, rate_hz_));
    timer_ = this->create_wall_timer(period, [this]() {
      // Fault injection: delay
      if (fault_delay_us_ > 0) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count() < fault_delay_us_) {
          // busy wait
        }
      }
      // Fault injection: drop Nth
      if (fault_drop_n_ > 0 && (++counter_ % fault_drop_n_ == 0)) {
        return;
      }
      std_msgs::msg::Header msg;
      msg.stamp = this->now();
      msg.frame_id = "tick";
      pub_->publish(msg);
    });
  }

  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr degrade_srv_, restore_srv_;

  // Config
  std::string reliability_, history_;
  int depth_, deadline_ms_, latency_budget_ms_, rate_hz_;

  // Fault injection
  int fault_drop_n_, fault_delay_us_;
  uint64_t counter_{0};
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QoSPublisher>());
  rclcpp::shutdown();
  return 0;
}
