#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/service.hpp>
#include <string>
#include "door_interface/srv/door_control.hpp"
#include "../include/cmd_sender.hpp"

class DoorControlNode : public rclcpp::Node
{
  public:
    DoorControlNode() : Node("door_control_node")
    {

      RCLCPP_INFO(this->get_logger(), "Door Control Node has been initialized.");
      door_service_ = this->create_service<door_interface::srv::DoorControl>(
        "door_control",
        std::bind(&DoorControlNode::control_door, this, std::placeholders::_1, std::placeholders::_2));
    }


  private:
   SerialSender serial_sender_{"/dev/ttyUSB0"}; // Initialize SerialSender with the appropriate port
   int box_id_;
   int box_status_;
   int door_cmd_;
   int door_status_;
   std::string data;
   rclcpp::Service<door_interface::srv::DoorControl>::SharedPtr door_service_;
   void control_door(const std::shared_ptr<door_interface::srv::DoorControl::Request> request,
                     std::shared_ptr<door_interface::srv::DoorControl::Response> response)
   {
     RCLCPP_INFO(this->get_logger(), "Received door control request: box_id=%d, box_status=%d, door_cmd=%d",
                 request->box_id, request->box_status, request->door_cmd);
      this->box_id_ = request->box_id;
      this->box_status_ = request->box_status;
      this->door_cmd_ = request->door_cmd;
      this->data = std::to_string(this->box_id_)+ "," + std::to_string(this->box_status_) + "," + std::to_string(this->door_cmd_);
      door_status_ = serial_sender_.sendData(this->data);//Send command and get door status
     
     response->door_status = door_status_;
     RCLCPP_INFO(this->get_logger(), "Door status updated to: %d", response->door_status);

   };


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