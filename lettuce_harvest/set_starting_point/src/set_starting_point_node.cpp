#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <xarm_msgs/srv/move_cartesian.hpp>
#include <xarm_msgs/srv/set_digital_io.hpp>

#include "origin_coordinate_msgs/msg/origin_coordinates.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SetStartingPoint : public rclcpp::Node {
 public:
  SetStartingPoint() : Node("set_starting_point", rclcpp::NodeOptions()){
    arm_client = this->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");
    end_effector_client = this-> create_client<xarm_msgs::srv::SetDigitalIO>("/xarm/set_tgpio_digital");

    auto_open_close(end_effector_client, false); //電源を入れると勝手に開くので閉じさせる

    operational_subscription = this->create_subscription<std_msgs::msg::String>(
      "move_origin_topic", 10, std::bind(&SetStartingPoint::topic_callback, this, std::placeholders::_1)
    );


    //create client


    

    std::cout << "set_starting_point node is beginning..." << std::endl;
    message_received = false;
  
  }
  
 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operational_subscription;

  rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr arm_client;
  rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr end_effector_client;

  bool message_received;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback() {
    if (message_received) {
      move_robot_arm(arm_client, "skip"); // Replace with the desired operational_msg
    }
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: %s", msg->data.c_str());
    message_received = true;
    move_robot_arm(arm_client, msg->data.c_str());
  }  

  //ロボットアームの動作生成
  void move_robot_arm(rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr client, std::string operational_msg) const {
    if(!message_received) {
      RCLCPP_WARN(this->get_logger(),"メッセージ受信なし。スキップします");
      return;
    }
    if (operational_msg == "skip") {
      return;
    }

    auto request = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();

    std::vector<float> point = {300, 0, 250, 3.14, 0, 0};

    if (operational_msg == "x_up") {
      point[0] += 20;
    } else if (operational_msg == "x_down") {
      point[0] -= 20;
    } else if (operational_msg == "y_up") {
      point[1] += 20;
    } else if (operational_msg == "y_down") {
      point[1] -= 20;
    } //else if (operational_msg == "set_complete") {

    //} else {

    //}

    request->pose = point;
    request->speed = 150.0;
    request->acc = 50.0;
    request->mvtime = 0.0;

    while(!client -> wait_for_service(1s)) {
      std::cout << "while" << std::endl;
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"サービス待機中に中断されました。終了します。");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"サービスが利用できません、再度待機中");
    }

    auto future = client -> async_send_request(request);
    auto status = future.wait_for(3.5s);

    if (status == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(),"Responce: %s", future.get() -> message.c_str());
    }
  }

  //エンドエフェクタの開閉
  void auto_open_close(rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr client, bool open_close_flag) {
    std::string show_msg = "";
    auto request = std::make_shared<xarm_msgs::srv::SetDigitalIO::Request>();
    request->ionum = 0;

    if (open_close_flag) {
      request->value = 0;
      show_msg = "open";
    } else {
      request->value = 1;
      show_msg = "close";
    }

    while(!client->wait_for_service(1s)) {
      if(!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"サービス待機中に中断されました。終了します。");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"サービスが利用できません、再度待機中");
    }

    std::cout << show_msg << std::endl;
    auto result = client->async_send_request(request);
  }
};

int main (int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetStartingPoint>());
  rclcpp::shutdown();
  return 0;
}
