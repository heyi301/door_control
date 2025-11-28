

#include "../include/cmd_sender.hpp"
#include "common/ros2_sport_client.h"
#include "door_interface/action/door_control.hpp"
#include "door_interface/srv/door_control.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include <chrono>
#include <cmath>
#include <door_interface/action/detail/door_control__struct.hpp>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <unitree_go/msg/detail/sport_mode_state__struct.hpp>

#define TOPIC_HIGHSTATE "lf/sportmodestate"
class DoorControlNode : public rclcpp::Node {
public:
  explicit DoorControlNode() : Node("door_control_node"), sport_client_(this) {

    RCLCPP_INFO(this->get_logger(), "Door Control Node has been initialized.");
    door_service_ = this->create_service<door_interface::srv::DoorControl>(
        "door_control",
        std::bind(&DoorControlNode::control_door, this, std::placeholders::_1,
                  std::placeholders::_2));
    door_action_service_ =
        rclcpp_action::create_server<door_interface::action::DoorControl>(
            this, "door_control_action",
            [this](const auto &uuid, auto goal) {
              return this->goal(uuid, goal);
            },
            [this](const auto goal_handle) {
              return this->cancel(goal_handle);
            },
            [this](const auto goal_handle) { this->accept(goal_handle); }

        );

    RCLCPP_INFO(this->get_logger(), "服务器启动成功");
    suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
        TOPIC_HIGHSTATE, 1,
        [this](const unitree_go::msg::SportModeState::SharedPtr data) {
          HighStateHandler(data);
        });
  }

  void GetInitState() {
    px0_ = state_.position[0];
    py0_ = state_.position[1];
    yaw0_ = state_.imu_state.rpy[2];
    RCLCPP_INFO(this->get_logger(),
                "initial position: x0: %f, y0: %f, yaw0: %f", px0_, py0_,
                yaw0_);
  }

  void HighStateHandler(const unitree_go::msg::SportModeState::SharedPtr msg) {
    state_ = *msg;
    // RCLCPP_INFO(this->get_logger(), "Position: %f, %f, %f", state_.position[0],
    //             state_.position[1], state_.position[2]);
    // RCLCPP_INFO(this->get_logger(), "IMU rpy: %f, %f, %f",
    //             state_.imu_state.rpy[0], state_.imu_state.rpy[1],
    //             state_.imu_state.rpy[2]);
  }

  void control_door(
      const std::shared_ptr<door_interface::srv::DoorControl::Request> request,
      std::shared_ptr<door_interface::srv::DoorControl::Response> response) {
    RCLCPP_INFO(
        this->get_logger(),
        "Received door control request: box_id=%d, box_status=%d, door_cmd=%d",
        request->box_id, request->box_status, request->door_cmd);
    this->box_id_ = request->box_id;
    this->box_status_ = request->box_status;
    this->door_cmd_ = request->door_cmd;
    this->data = std::to_string(this->box_id_) +
                 std::to_string(this->box_status_) +
                 std::to_string(this->door_cmd_) + "#";
    door_status_ =
        serial_sender_.sendData(this->data); // Send command and get door status

    response->door_status = door_status_;
    // RCLCPP_INFO(this->get_logger(), "Door status updated to: %d",
    // response->door_status);
  };

  rclcpp_action::GoalResponse
  goal(const rclcpp_action::GoalUUID &uuid,
       std::shared_ptr<const door_interface::action::DoorControl::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "收到请求:box_id = %d , box_cmd = %d",
                this->box_id_, this->door_cmd_);
    this->box_id_ = goal->box_id;
    this->door_cmd_ = goal->door_cmd;
    this->data = std::to_string(this->box_id_) +
                 std::to_string(this->box_status_) +
                 std::to_string(this->door_cmd_);
    // door_status_ = serial_sender_.sendData(this->data);
    (void)uuid; // 避免未使用参数警告
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  cancel(std::shared_ptr<
         rclcpp_action::ServerGoalHandle<door_interface::action::DoorControl>>
             door_control) {
    RCLCPP_INFO(this->get_logger(), "收到取消请求");

    // 这里可以添加取消逻辑，比如停止计算等
    (void)door_control; // 避免未使用参数警告

    // 接受取消请求
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void
  accept(std::shared_ptr<
         rclcpp_action::ServerGoalHandle<door_interface::action::DoorControl>>
             door_control) {
    RCLCPP_INFO(this->get_logger(), "开始执行目标");
    std::thread{std::bind(&DoorControlNode::execute_action, this,
                          std::placeholders::_1),
                door_control}
        .detach();
  }

  void execute_action(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<door_interface::action::DoorControl>>
          door_control) {
    RCLCPP_INFO(this->get_logger(), "开始执行倾倒动作");
    int box_id = door_control->get_goal()->box_id;
    auto feedback_msg = std::make_shared<
        typename door_interface::action::DoorControl::Feedback>();
    feedback_msg->current_action = 0;
    door_control->publish_feedback(feedback_msg);
    if (box_id == 1) {
       // jie锁
      this->data = std::to_string(this->box_id_) +
                 std::to_string(this->box_status_) +
                 std::to_string(0) + "#";
      door_status_ =
        serial_sender_.sendData(this->data); // Send command and get door status
      // sport_client_.StandUp(req_);
      sport_client_.Euler(req_, -0.8, 0, 0);
      sport_client_.BalanceStand(req_);
      usleep(int(1000000));
      sport_client_.Euler(req_, -0.8, 0, 0);
      sport_client_.BalanceStand(req_);
      usleep(int(1000000));
      RCLCPP_INFO(this->get_logger(), "倾斜");
      // 执行倾倒动作
      feedback_msg->current_action = 1;
      door_control->publish_feedback(feedback_msg);
      // 上锁
      this->data = std::to_string(this->box_id_) +
                 std::to_string(this->box_status_) +
                 std::to_string(1) + "#";
      door_status_ =
        serial_sender_.sendData(this->data); // Send command and get door status
      feedback_msg->current_action = 2;
      door_control->publish_feedback(feedback_msg);
      // 执行回正动作并略微超调
      RCLCPP_INFO(this->get_logger(), "回正");
      sport_client_.Euler(req_, 0.5, 0, 0);
      sport_client_.BalanceStand(req_);
      usleep(int(1000000));
      sport_client_.RecoveryStand(req_);
      feedback_msg->current_action = 3;
      door_control->publish_feedback(feedback_msg);
      auto result = std::make_shared<
          typename door_interface::action::DoorControl::Result>();
      result->result = door_status_;
      door_control->succeed(result);
    } else {
       // 上锁
      this->data = std::to_string(this->box_id_) +
                 std::to_string(this->box_status_) +
                 std::to_string(0) + "#";
      door_status_ =
        serial_sender_.sendData(this->data); // Send command and get door status
      // sport_client_.StandUp(req_);
      sport_client_.Euler(req_, 1.2, 0, 0);
      sport_client_.BalanceStand(req_);
      usleep(int(1000000));
      sport_client_.Euler(req_, 1.2, 0, 0);
      sport_client_.BalanceStand(req_);
      usleep(int(1000000));
      RCLCPP_INFO(this->get_logger(), "倾斜");
      // 执行倾倒动作
      feedback_msg->current_action = 1;
      door_control->publish_feedback(feedback_msg);
      // 上锁
      this->data = std::to_string(this->box_id_) +
                 std::to_string(this->box_status_) +
                 std::to_string(1) + "#";
      door_status_ =
        serial_sender_.sendData(this->data); // Send command and get door status
      feedback_msg->current_action = 2;
      door_control->publish_feedback(feedback_msg);
      // 执行回正动作并略微超调
      sport_client_.Euler(req_, -0.5, 0, 0);
      sport_client_.BalanceStand(req_);
      usleep(int(1000000));
      sport_client_.RecoveryStand(req_);
      feedback_msg->current_action = 3;
      door_control->publish_feedback(feedback_msg);
      auto result = std::make_shared<
          typename door_interface::action::DoorControl::Result>();
      result->result = door_status_;
      door_control->succeed(result);
    }
  }
  SerialSender serial_sender_{
      "/dev/ttyBOX1"}; // Initialize SerialSender with the appropriate port
private:
  
  int box_id_;
  int box_status_;
  int door_cmd_;
  int door_status_;
  std::string data;

  rclcpp::Service<door_interface::srv::DoorControl>::SharedPtr door_service_;
  rclcpp_action::Server<door_interface::action::DoorControl>::SharedPtr
      door_action_service_;

  unitree_go::msg::SportModeState state_;
  SportClient sport_client_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr suber_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr req_suber_;
  unitree_api::msg::Request req_;
  double px0_{}, py0_{}, yaw0_{};
  double ct_{};
  int flag_{};
  float dt_ = 0.1;
  int test_mode_;
  std::thread t1_;

  
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DoorControlNode>();
  node->GetInitState();
  RCLCPP_INFO(node->get_logger(), "Control Door Node has started.");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::string init_data=std::to_string(2) +
                 std::to_string(1) +
                 std::to_string(1) + "#";
  node->serial_sender_.sendData(init_data); // Send command and get door status
  executor.spin();
  rclcpp::shutdown();
  return 0;
}