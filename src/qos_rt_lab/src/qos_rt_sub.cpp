#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <algorithm>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
using namespace std::chrono_literals;
struct Stats{ uint64_t rx=0, lost=0; std::vector<double> lat_ms, period_us; rclcpp::Time last; };
class Sub : public rclcpp::Node{
public:
  Sub(): Node("qos_rt_sub"), up_(this){
    reliability_=declare_parameter<std::string>("reliability","reliable");
    history_=declare_parameter<std::string>("history","keep_last");
    depth_=declare_parameter<int>("depth",10);
    run_id_=declare_parameter<std::string>("run_id","local");
    out_dir_=declare_parameter<std::string>("out_dir","results");
    lat_warn_=declare_parameter<double>("lat_p95_warn_ms",3.0);
    lat_err_=declare_parameter<double>("lat_p95_error_ms",6.0);
    jit_warn_=declare_parameter<double>("jitter_warn_us",150.0);
    jit_err_=declare_parameter<double>("jitter_error_us",400.0);
    loss_warn_=declare_parameter<double>("loss_warn",0.001);
    loss_err_=declare_parameter<double>("loss_error",0.01);
    rclcpp::QoS qos=(history_=="keep_all")? rclcpp::QoS(rclcpp::KeepAll()): rclcpp::QoS(rclcpp::KeepLast(depth_));
    if (reliability_=="best_effort") qos.best_effort(); else qos.reliable();
    sub_=create_subscription<std_msgs::msg::Header>("qos_rt/tick",qos,[this](auto m){ on_msg(m);});
    t_=create_wall_timer(2s,[this]{log(false);}); up_.setHardwareID("qos_rt_chain"); up_.add("QoS Link Health",this,&Sub::diag);
  }
  ~Sub(){ try{ log(true);}catch(...){} }
private:
  static double mean(const std::vector<double>& v){ if(v.empty())return 0; double s=0; for(double x:v)s+=x; return s/v.size(); }
  static double perc(std::vector<double> v,double p){ if(v.empty())return 0; std::sort(v.begin(),v.end()); size_t i=(size_t)std::ceil(p*v.size())-1; if(i>=v.size())i=v.size()-1; return v[i]; }
  static double stdev(const std::vector<double>& v){ if(v.size()<2)return 0; double m=mean(v),s=0; for(double x:v){double d=x-m;s+=d*d;} return std::sqrt(s/(v.size()-1)); }
  void on_msg(const std_msgs::msg::Header::SharedPtr m){
    auto now=this->now(); st_.lat_ms.push_back((now - m->stamp).seconds()*1000.0); st_.rx++;
    if(st_.last.nanoseconds()>0){ double per=(m->stamp - st_.last).seconds()*1e6; st_.period_us.push_back(per); if(per>1500.0) st_.lost+= (uint64_t)std::round(per/1000.0)-1; }
    st_.last=m->stamp; up_.force_update();
  }
  void diag(diagnostic_updater::DiagnosticStatusWrapper& s){
    double p95=perc(st_.lat_ms,0.95), jit=stdev(st_.period_us), loss=(st_.rx+st_.lost)?(double)st_.lost/(st_.rx+st_.lost):0.0;
    int lvl=0; std::string msg="OK";
    if(p95>lat_err_||jit>jit_err_||loss>loss_err_){ lvl=2; msg="ERROR"; }
    else if(p95>lat_warn_||jit>jit_warn_||loss>loss_warn_){ lvl=1; msg="WARN"; }
    s.summary(lvl,msg); s.add("received",(int)st_.rx); s.add("lost",(int)st_.lost); s.add("lat_p95_ms",p95); s.add("jitter_us",jit); s.add("loss_rate",loss);
  }
  void log(bool final){
    double p95=perc(st_.lat_ms,0.95), jit=stdev(st_.period_us), loss=(st_.rx+st_.lost)?(double)st_.lost/(st_.rx+st_.lost):0.0;
    RCLCPP_INFO(this->get_logger(), "%s | rx=%lu lost=%lu | p95(ms)=%.3f jitter(us)=%.2f loss=%.4f",final?"FINAL":"LIVE",(unsigned long)st_.rx,(unsigned long)st_.lost,p95,jit,loss);
    if(final){ std::filesystem::create_directories(out_dir_); std::string path=out_dir_+"/qos_rt_"+run_id_+".json"; std::ofstream f(path); f<<"{\\n  \\\"received\\\": "<<st_.rx<<", \\n  \\\"lost\\\": "<<st_.lost<<", \\n  \\\"lat_p95_ms\\\": "<<p95<<", \\n  \\\"jitter_us\\\": "<<jit<<", \\n  \\\"loss_rate\\\": "<<loss<<"\\n}\\n"; }
  }
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_; rclcpp::TimerBase::SharedPtr t_; diagnostic_updater::Updater up_;
  Stats st_; std::string reliability_,history_,run_id_,out_dir_; int depth_; double lat_warn_,lat_err_,jit_warn_,jit_err_,loss_warn_,loss_err_;
};
int main(int argc,char**argv){ rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<Sub>()); rclcpp::shutdown(); return 0; }
