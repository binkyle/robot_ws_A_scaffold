
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

using namespace std::chrono_literals;

struct Stats {
  uint64_t received{0};
  uint64_t lost{0};
  std::vector<double> lat_ms;
  std::vector<double> period_us;
  rclcpp::Time last_stamp;
};

class QoSSubscriber : public rclcpp::Node {
public:
  QoSSubscriber() : Node("qos_rt_sub"), updater_(this) {
    // Params
    reliability_ = declare_parameter<std::string>("reliability", "reliable");
    history_ = declare_parameter<std::string>("history", "keep_last");
    depth_ = declare_parameter<int>("depth", 10);
    deadline_ms_ = declare_parameter<int>("deadline_ms", 0);
    latency_budget_ms_ = declare_parameter<int>("latency_budget_ms", 0);
    run_id_ = declare_parameter<std::string>("run_id", "local");
    out_dir_ = declare_parameter<std::string>("out_dir", "results");

    lat_p95_warn_ms_ = declare_parameter<double>("lat_p95_warn_ms", 3.0);
    lat_p95_error_ms_ = declare_parameter<double>("lat_p95_error_ms", 6.0);
    jitter_warn_us_ = declare_parameter<double>("jitter_warn_us", 150.0);
    jitter_error_us_ = declare_parameter<double>("jitter_error_us", 400.0);
    loss_warn_ = declare_parameter<double>("loss_warn", 0.001);  // 0.1%
    loss_error_ = declare_parameter<double>("loss_error", 0.01); // 1%

    // QoS
    rclcpp::QoS qos = (history_ == "keep_all") ? rclcpp::QoS(rclcpp::KeepAll())
                                               : rclcpp::QoS(rclcpp::KeepLast(depth_));
    if (reliability_ == "best_effort") qos.best_effort(); else qos.reliable();
    if (deadline_ms_ > 0) qos.deadline(std::chrono::milliseconds(deadline_ms_));
    if (latency_budget_ms_ > 0) qos.latency_budget(std::chrono::milliseconds(latency_budget_ms_));

    sub_ = this->create_subscription<std_msgs::msg::Header>("qos_rt/tick", qos,
      std::bind(&QoSSubscriber::cb, this, std::placeholders::_1));

    log_timer_ = this->create_wall_timer(2s, [this](){ log_stats(false); });

    updater_.setHardwareID("qos_rt_chain");
    updater_.add("QoS Link Health", this, &QoSSubscriber::diag_cb);

    RCLCPP_INFO(this->get_logger(), "qos_rt_sub running.");
  }

  ~QoSSubscriber() override {
    try { log_stats(true); } catch (...) {}
  }

private:
  void cb(const std_msgs::msg::Header::SharedPtr msg) {
    auto now = this->now();
    double lat = (now - msg->stamp).seconds()*1000.0;
    st_.lat_ms.push_back(lat);
    st_.received++;

    if (st_.last_stamp.nanoseconds() > 0) {
      double period = (msg->stamp - st_.last_stamp).seconds() * 1e6;
      st_.period_us.push_back(period);
      if (period > 1500.0) {
        st_.lost += static_cast<uint64_t>(std::round(period/1000.0) - 1.0);
      }
    }
    st_.last_stamp = msg->stamp;

    // Update diagnostics at message rate throttled by timer
    updater_.force_update();
  }

  static double mean(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    double s=0; for (auto& x: v) s+=x; return s/v.size();
  }
  static double percentile(std::vector<double> v, double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    size_t idx = static_cast<size_t>(std::ceil(p * v.size())) - 1;
    if (idx >= v.size()) idx = v.size()-1;
    return v[idx];
  }
  static double stdev(const std::vector<double>& v) {
    if (v.size()<2) return 0.0;
    double m = mean(v); double s=0;
    for (auto& x: v){ double d=x-m; s+=d*d; }
    return std::sqrt(s/(v.size()-1));
  }

  void diag_cb(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    // Compute quick metrics
    double lat_p95 = percentile(st_.lat_ms, 0.95);
    double lat_max = st_.lat_ms.empty() ? 0.0 : *std::max_element(st_.lat_ms.begin(), st_.lat_ms.end());
    double jit = stdev(st_.period_us);
    double loss_rate = 0.0;
    if (st_.received + st_.lost > 0) loss_rate = static_cast<double>(st_.lost) / (st_.received + st_.lost);

    int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string summary = "OK";
    if (lat_p95 > lat_p95_error_ms_ || jit > jitter_error_us_ || loss_rate > loss_error_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR; summary = "ERROR";
    } else if (lat_p95 > lat_p95_warn_ms_ || jit > jitter_warn_us_ || loss_rate > loss_warn_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN; summary = "WARN";
    }
    stat.summary(level, summary);
    stat.add("received", (int)st_.received);
    stat.add("lost", (int)st_.lost);
    stat.add("lat_p95_ms", lat_p95);
    stat.add("lat_max_ms", lat_p95 > 0 ? lat_max : 0.0);
    stat.add("jitter_us", jit);
    stat.add("loss_rate", loss_rate);
  }

  void log_stats(bool final) {
    double lat_mean = mean(st_.lat_ms);
    double lat_p95 = percentile(st_.lat_ms, 0.95);
    double lat_max = st_.lat_ms.empty() ? 0.0 : *std::max_element(st_.lat_ms.begin(), st_.lat_ms.end());
    double jit = stdev(st_.period_us);
    double loss_rate = (st_.received + st_.lost) ? (double)st_.lost / (st_.received + st_.lost) : 0.0;

    RCLCPP_INFO(this->get_logger(),
      "%s stats | recv=%lu lost=%lu | latency(ms): mean=%.3f p95=%.3f max=%.3f | jitter(us)=%.2f | loss=%.4f",
      final ? "FINAL" : "LIVE", (unsigned long)st_.received, (unsigned long)st_.lost,
      lat_mean, lat_p95, lat_max, jit, loss_rate);

    if (final) {
      std::error_code ec; std::filesystem::create_directories(out_dir_, ec);
      auto path = out_dir_ + "/qos_rt_" + run_id_ + ".json";
      std::ofstream f(path);
      f << "{\\n"
        << "  \\\"received\\\": " << st_.received << ",\\n"
        << "  \\\"lost\\\": " << st_.lost << ",\\n"
        << "  \\\"lat_mean_ms\\\": " << lat_mean << ",\\n"
        << "  \\\"lat_p95_ms\\\": " << lat_p95 << ",\\n"
        << "  \\\"lat_max_ms\\\": " << lat_max << ",\\n"
        << "  \\\"jitter_us\\\": " << jit << ",\\n"
        << "  \\\"loss_rate\\\": " << loss_rate << "\\n"
        << "}\\n";
      f.close();
      RCLCPP_INFO(this->get_logger(), "Summary written: %s", path.c_str());
    }
  }

  // QoS config
  std::string reliability_, history_; int depth_, deadline_ms_, latency_budget_ms_;
  std::string run_id_, out_dir_;
  double lat_p95_warn_ms_, lat_p95_error_ms_, jitter_warn_us_, jitter_error_us_, loss_warn_, loss_error_;

  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr log_timer_;
  diagnostic_updater::Updater updater_;
  Stats st_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QoSSubscriber>());
  rclcpp::shutdown();
  return 0;
}
