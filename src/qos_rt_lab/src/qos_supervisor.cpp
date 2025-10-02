#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
using namespace std::chrono_literals;
class Sup : public rclcpp::Node{
public:
  Sup(): Node("qos_supervisor"){
    d_warn_=declare_parameter<int>("degrade_after_warn_count",3);
    r_ok_=declare_parameter<int>("restore_after_ok_count",5);
    sub_=create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",10,[this](auto m){ on_diag(m); });
    c_deg_=create_client<std_srvs::srv::Trigger>("/qos_rt_pub/degrade");
    c_res_=create_client<std_srvs::srv::Trigger>("/qos_rt_pub/restore");
  }
private:
  void on_diag(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr m){
    int level=0; for(auto& s: m->status){ if(s.name.find("QoS Link Health")!=std::string::npos){ level=s.level; break; } }
    if(level==1 || level==2){ w_++; o_=0; if(!deg_ && w_>=d_warn_){ deg_=true; w_=0; call(c_deg_,"degrade"); } }
    else { o_++; w_=0; if(deg_ && o_>=r_ok_){ deg_=false; o_=0; call(c_res_,"restore"); } }
  }
  void call(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr c, const char* tag){
    if(!c->wait_for_service(1s)){ RCLCPP_WARN(get_logger(), "Service %s not ready", tag); return; }
    auto req=std::make_shared<std_srvs::srv::Trigger::Request>(); (void)c->async_send_request(req); RCLCPP_INFO(get_logger(),"Requested %s", tag);
  }
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr c_deg_, c_res_;
  int d_warn_{3}, r_ok_{5}, w_{0}, o_{0}; bool deg_{false};
};
int main(int argc,char**argv){ rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<Sup>()); rclcpp::shutdown(); return 0; }
