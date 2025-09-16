#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using namespace std::chrono_literals;

struct LcHandles {
  std::string name;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get;
};

class LifecycleController : public rclcpp::Node {
public:
  LifecycleController()
  : Node("lifecycle_controller")
  {
    talker_   = declare_parameter<std::string>("talker",   "/lifecycle_talker");
    listener_ = declare_parameter<std::string>("listener", "/lifecycle_listener");
    t_ = make_handles(talker_);
    l_ = make_handles(listener_);
    wait_services(t_);
    wait_services(l_);
    print_help();
  }

  void spin_menu() {
    for (;;) {
      std::cout << "\n입력(1:configure, 2:activate, 3:deactivate, 4:cleanup, 5:shutdown, s:state, h:help, q:quit) > " << std::flush;
      std::string line;
      if (!std::getline(std::cin, line)) break;
      if (line.empty()) continue;
      char c = line[0];
      if (c == 'q') break;
      if (c == 'h') { print_help(); continue; }
      if (c == 's') { log_state("talker", t_); log_state("listener", l_); continue; }

      uint8_t tr = 0;
      switch (c) {
        case '1': tr = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE; break;
        case '2': tr = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;  break;
        case '3': tr = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;break;
        case '4': tr = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;   break;
        case '5': tr = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN; break;
        default: std::cout << "알 수 없는 입력\n"; continue;
      }
      apply_both(tr);
    }
  }

private:
  LcHandles make_handles(const std::string & n) {
    LcHandles h;
    h.name = n;
    h.change = create_client<lifecycle_msgs::srv::ChangeState>(n + "/change_state");
    h.get    = create_client<lifecycle_msgs::srv::GetState>(n + "/get_state");
    return h;
  }

  void wait_services(const LcHandles & h) {
    std::cout << h.name << " 서비스 대기…" << std::endl;
    while (!h.change->wait_for_service(500ms) || !h.get->wait_for_service(500ms)) {
      if (!rclcpp::ok()) { throw std::runtime_error("interrupted"); }
    }
  }

  void apply_both(uint8_t transition_id) {
    // 두 노드 병렬 요청
    auto req = [](uint8_t id){
      auto r = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
      r->transition.id = id; return r;
    };
    auto f1 = t_.change->async_send_request(req(transition_id));
    auto f2 = l_.change->async_send_request(req(transition_id));
    auto ok1 = f1.wait_for(3s) == std::future_status::ready && f1.get()->success;
    auto ok2 = f2.wait_for(3s) == std::future_status::ready && f2.get()->success;
    std::cout << "[talker] "   << (ok1 ? "OK" : "FAIL") << "\n";
    std::cout << "[listener] " << (ok2 ? "OK" : "FAIL") << "\n";
    log_state("talker", t_);
    log_state("listener", l_);
  }

  void log_state(const std::string & tag, LcHandles & h) {
    auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto fut = h.get->async_send_request(req);
    if (fut.wait_for(2s) != std::future_status::ready) {
      std::cout << "[" << tag << "] state: TIMEOUT\n"; return;
    }
    auto id = fut.get()->current_state.id;
    std::cout << "[" << tag << "] state: " << state_str(id) << " (" << unsigned(id) << ")\n";
  }

  static const char* state_str(uint8_t id) {
    using lifecycle_msgs::msg::State;
    switch (id) {
      case State::PRIMARY_STATE_UNCONFIGURED: return "unconfigured";
      case State::PRIMARY_STATE_INACTIVE:     return "inactive";
      case State::PRIMARY_STATE_ACTIVE:       return "active";
      case State::PRIMARY_STATE_FINALIZED:    return "finalized";
      default: return "unknown";
    }
  }

  void print_help() {
    std::cout <<
      "\n[메뉴]\n"
      " 1: configure  2: activate  3: deactivate  4: cleanup  5: shutdown\n"
      " s: 상태조회    h: 도움말    q: 종료\n"
      " 대상: " << talker_ << " , " << listener_ << "\n";
  }

  std::string talker_, listener_;
  LcHandles t_, l_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<LifecycleController>();
    node->spin_menu();
  } catch (const std::exception & e) {
    std::cerr << "에러: " << e.what() << "\n";
  }
  rclcpp::shutdown();
  return 0;
}
