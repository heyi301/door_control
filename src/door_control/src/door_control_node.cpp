#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <door_interface/action/detail/door_control__struct.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <string>
#include "door_interface/srv/door_control.hpp"
#include "../include/cmd_sender.hpp"
#include "door_interface/action/door_control.hpp"
#include <thread>

class DoorControlNode : public rclcpp::Node
{
  public:
    DoorControlNode() : Node("door_control_node")
    {

      RCLCPP_INFO(this->get_logger(), "Door Control Node has been initialized.");
      door_service_ = this->create_service<door_interface::srv::DoorControl>(
        "door_control",
        std::bind(&DoorControlNode::control_door, this, std::placeholders::_1, std::placeholders::_2));
      door_action_service_ = rclcpp_action::create_server<door_interface::action::DoorControl>(
        this,
      "door_control_action",
      [this](const auto& uuid, auto goal) {
        return this->goal(uuid, goal);
      },
      [this](const auto goal_handle) {
        return this->cancel(goal_handle);
      },
      [this](const auto goal_handle) {
        this->accept(goal_handle);
      }
      
    );
    RCLCPP_INFO(this->get_logger(), "服务器启动成功");
    }


  private:
   SerialSender serial_sender_{"/dev/ttyBOX1"}; // Initialize SerialSender with the appropriate port
   int box_id_;
   int box_status_;
   int door_cmd_;
   int door_status_;
   std::string data;
   rclcpp::Service<door_interface::srv::DoorControl>::SharedPtr door_service_;
   rclcpp_action::Server<door_interface::action::DoorControl>::SharedPtr door_action_service_;
   void control_door(const std::shared_ptr<door_interface::srv::DoorControl::Request> request,
                     std::shared_ptr<door_interface::srv::DoorControl::Response> response)
   {
     RCLCPP_INFO(this->get_logger(), "Received door control request: box_id=%d, box_status=%d, door_cmd=%d",
                 request->box_id, request->box_status, request->door_cmd);
      this->box_id_ = request->box_id;
      this->box_status_ = request->box_status;
      this->door_cmd_ = request->door_cmd;
      this->data = std::to_string(this->box_id_) + std::to_string(this->box_status_) + std::to_string(this->door_cmd_) + "#";
      door_status_ = serial_sender_.sendData(this->data);//Send command and get door status
     
      response->door_status = door_status_;
      // RCLCPP_INFO(this->get_logger(), "Door status updated to: %d", response->door_status);

   };
   rclcpp_action::GoalResponse goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const door_interface::action::DoorControl::Goal> goal){
    RCLCPP_INFO(this->get_logger(),"收到请求:box_id = %d , box_cmd = %d",this->box_id_,this->door_cmd_);
    this->box_id_=goal->box_id;
    this->door_cmd_=goal->door_cmd;
    this->data = std::to_string(this->box_id_) + std::to_string(this->box_status_)+std::to_string(this->door_cmd_);
    door_status_= serial_sender_.sendData(this->data);
    (void)uuid; // 避免未使用参数警告
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
   }
   rclcpp_action::CancelResponse cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<door_interface::action::DoorControl>> door_control){
    RCLCPP_INFO(this->get_logger(), "收到取消请求");
    
    // 这里可以添加取消逻辑，比如停止计算等
    (void)door_control; // 避免未使用参数警告
    
    // 接受取消请求
    return rclcpp_action::CancelResponse::ACCEPT;
   }
   void accept(std::shared_ptr<rclcpp_action::ServerGoalHandle<door_interface::action::DoorControl>> door_control)
   {
    RCLCPP_INFO(this->get_logger(), "开始执行目标");
    std::thread{
      std::bind(&DoorControlNode::execute_action, this, std::placeholders::_1),
      door_control}.detach();
    
   }
   void execute_action(const std::shared_ptr<rclcpp_action::ServerGoalHandle<door_interface::action::DoorControl>> door_control){
    RCLCPP_INFO(this->get_logger(), "开始执行倾倒动作");
    int box_id = door_control->get_goal()->box_id;
    auto feedback_msg = std::make_shared<typename door_interface::action::DoorControl::Feedback>();
    feedback_msg->current_action=0;
    door_control->publish_feedback(feedback_msg);
    if(box_id == 0)
    {
      //执行倾倒动作
      feedback_msg->current_action=1;
      door_control->publish_feedback(feedback_msg);
      //上锁
      feedback_msg->current_action=2;
      door_control->publish_feedback(feedback_msg);
      //执行回正动作并略微超调
      feedback_msg->current_action=3;
      door_control->publish_feedback(feedback_msg);
      auto result = std::make_shared<typename door_interface::action::DoorControl::Result>();
      result->result = door_status_;
      door_control->succeed(result);
    }
    else{
      //执行倾倒动作
      //上锁
      //执行回正动作并略微超调
    }
   }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DoorControlNode>();
  RCLCPP_INFO(node->get_logger(), "Control Door Node has started.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}