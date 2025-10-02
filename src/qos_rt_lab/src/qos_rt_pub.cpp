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
    reliability_ = declare_parameter<std::string>("reliability", "reliable");
    history_ = declare_parameter<std::string>("history", "keep_last");
    depth_ = declare_parameter<int>("depth", 10);
    rate_hz_ = declare_parameter<int>("rate_hz", 1000);
    fault_drop_n_ = declare_parameter<int>("fault_drop_n", 0);
    fault_delay_us_ = declare_parameter<int>("fault_delay_us", 0);
    pristine_ = {reliability_, history_, depth_, rate_hz_};
    recreate_pub(); create_timer();
    degrade_srv_ = create_service<std_srvs::srv::Trigger>("degrade",
      [this](auto, auto res){ reliability_="best_effort"; history_="keep_last"; depth_=std::min(depth_,5); rate_hz_=std::min(rate_hz_,200); recreate_pub(); create_timer(); res->success=true; res->message="Degraded"; });
    restore_srv_ = create_service<std_srvs::srv::Trigger>("restore",
      [this](auto, auto res){ reliability_=pristine_.rel; history_=pristine_.hist; depth_=pristine_.depth; rate_hz_=pristine_.rate; recreate_pub(); create_timer(); res->success=true; res->message="Restored"; });
  }
private:
  struct P{std::string rel,hist; int depth,rate;} pristine_;
  void recreate_pub(){
    rclcpp::QoS qos = (history_=="keep_all")? rclcpp::QoS(rclcpp::KeepAll()): rclcpp::QoS(rclcpp::KeepLast(depth_));
    if (reliability_=="best_effort") qos.best_effort(); else qos.reliable();
    pub_ = create_publisher<std_msgs::msg::Header>("qos_rt/tick", qos);
  }
  void create_timer(){
    if (timer_) timer_->cancel();
    auto period = std::chrono::microseconds(1000000 / std::max(1, rate_hz_));
    timer_ = create_wall_timer(period, [this](){
      if (fault_delay_us_>0){ auto s=std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-s).count()<fault_delay_us_){} }
      if (fault_drop_n_>0 && (++counter_%fault_drop_n_==0)) return;
      std_msgs::msg::Header m; m.stamp=now(); m.frame_id="tick"; pub_->publish(m);
    });
  }
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_; rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr degrade_srv_, restore_srv_;
  std::string reliability_, history_; int depth_, rate_hz_, fault_drop_n_, fault_delay_us_; uint64_t counter_{0};
};
int main(int argc, char** argv){ rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<QoSPublisher>()); rclcpp::shutdown(); return 0; }
