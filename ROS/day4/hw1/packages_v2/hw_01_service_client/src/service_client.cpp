#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

struct Lc {
  std::string name;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get;
};

class Controller : public rclcpp::Node {
public:
  Controller() : Node("lifecycle_controller") {
    talker_   = declare_parameter<std::string>("talker",   "/lifecycle_talker");
    listener_ = declare_parameter<std::string>("listener", "/lifecycle_listener");
    t_ = make(talker_);
    l_ = make(listener_);
    wait(t_);
    wait(l_);
    help();
  }

  void loop() {
    for (;;) {
      std::cout << "\n입력(1:configure 2:activate 3:deactivate 4:cleanup 5:shutdown s:state h:help q:quit)> " << std::flush;
      std::string s; if (!std::getline(std::cin, s)) break; if (s.empty()) continue;
      char c = s[0];
      if (c=='q') break;
      if (c=='h') { help(); continue; }
      if (c=='s') { show(); continue; }
      uint8_t tr=0;
      switch(c){
        case '1': tr=lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE; break;
        case '2': tr=lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE; break;
        case '3': tr=lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE; break;
        case '4': tr=lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP; break;
        case '5': tr=lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN; break;
        default: std::cout<<"unknown\n"; continue;
      }
      apply_both(tr);
    }
  }

private:
  Lc make(const std::string& n){
    Lc h; h.name=n;
    h.change = create_client<lifecycle_msgs::srv::ChangeState>(n + "/change_state");
    h.get    = create_client<lifecycle_msgs::srv::GetState>(n + "/get_state");
    return h;
  }
  void wait(const Lc& h){
    std::cout<<h.name<<" 서비스 대기…\n";
    while(!h.change->wait_for_service(500ms) || !h.get->wait_for_service(500ms)){
      if(!rclcpp::ok()) throw std::runtime_error("interrupted");
    }
  }
  void apply_both(uint8_t id){
    auto mk=[&](uint8_t x){ auto r=std::make_shared<lifecycle_msgs::srv::ChangeState::Request>(); r->transition.id=x; return r; };
    auto f1=t_.change->async_send_request(mk(id));
    auto f2=l_.change->async_send_request(mk(id));
    auto ok1=f1.wait_for(3s)==std::future_status::ready && f1.get()->success;
    auto ok2=f2.wait_for(3s)==std::future_status::ready && f2.get()->success;
    std::cout<<"[talker] "<<(ok1?"OK":"FAIL")<<"\n";
    std::cout<<"[listener] "<<(ok2?"OK":"FAIL")<<"\n";
    show();
  }
  static const char* sname(uint8_t id){
    using lifecycle_msgs::msg::State;
    switch(id){
      case State::PRIMARY_STATE_UNCONFIGURED: return "unconfigured";
      case State::PRIMARY_STATE_INACTIVE:     return "inactive";
      case State::PRIMARY_STATE_ACTIVE:       return "active";
      case State::PRIMARY_STATE_FINALIZED:    return "finalized";
      default: return "unknown";
    }
  }
  void show(){
    auto req=std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto f1=t_.get->async_send_request(req), f2=l_.get->async_send_request(req);
    if(f1.wait_for(2s)==std::future_status::ready) std::cout<<"[talker] "<<sname(f1.get()->current_state.id)<<"\n";
    if(f2.wait_for(2s)==std::future_status::ready) std::cout<<"[listener] "<<sname(f2.get()->current_state.id)<<"\n";
  }
  void help(){
    std::cout<<"\n[메뉴] 1:configure 2:activate 3:deactivate 4:cleanup 5:shutdown s:state h:help q:quit\n";
    std::cout<<"대상: "<<talker_<<" , "<<listener_<<"\n";
  }

  std::string talker_, listener_;
  Lc t_, l_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  try{ std::make_shared<Controller>()->loop(); }
  catch(const std::exception& e){ std::cerr<<"error: "<<e.what()<<"\n"; }
  rclcpp::shutdown();
  return 0;
}
