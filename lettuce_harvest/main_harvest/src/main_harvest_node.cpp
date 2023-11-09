#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <xarm_msgs/srv/move_cartesian.hpp>
#include <xarm_msgs/srv/set_digital_io.hpp>

#include "origin_coordinate_msgs/msg/origin_coordinates.hpp"

using namespace std::chrono_literals;

class MainHarvest : public rclcpp::Node {
 public:
  MainHarvest() : Node("main_harvest", rclcpp::NodeOptions()){
    arm_client = this->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");
    end_effector_client = this-> create_client<xarm_msgs::srv::SetDigitalIO>("/xarm/set_tgpio_digital");

    auto_open_close(end_effector_client, false); //電源を入れると勝手に開くので閉じさせる

    coordinates_subscription = this->create_subscription<origin_coordinate_msgs::msg::origin_coordinates>(
      "origin_coordinate_topic", 10, std::bind(&SetStartingPoint::topic_callback, this, std::placeholders::_1)
    );

    std::vector<std::vector<float>> origin_point = {x_origin, y_origin, z_origin, 3.14, 0, 0};

    for (int i = 0; i < 3; i++) {
      float y;

      if (i == 1) {
        y = y_origin + 30;
      } else {
        y = installation_point_list[i - 1] + 50;
      }

      std::vector<float> point = {x_origin, y, z_origin, 3.14, 0, 0};
      installation_point_list.push_back(point);
    }

    //move_arm(arm_client, origin_point); //move origin point befor harvest


    

    std::cout << "set_starting_point node is beginning..." << std::endl;
    message_received = false;
  
  }
  
 private:
  float x_origin;
  float y_origin;
  float z_origin;

  rclcpp::Subscription<origin_coordinate_msgs::msg::origin_coordinates>::SharedPtr coordinates_subscription;

  rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr arm_client;
  rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr end_effector_client;

  bool message_received;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::vector<float>> origin_point;
  std::vector<std::vector<float>> installation_point_list;
  const std::vector<std::vector<float>> packing_mechanism_point_list = {
    {200, 0, 300, 3.14, 0, 0},
    {100, 385, 108, 3.14, 0, -0.05},
    {235, 385, 108, 3.14, 0, -0.05},
    {235, 385, 98, 3.14, 0, -0.05},
    {235, 385, 98, 3.14, 0, -0.04},
    {235, 385, 200, 3.14, 0, -0.05} 
  }; //tmp

  void timer_callback() {
    if (message_received) {
      move_arm(arm_client, "skip"); // Replace with the desired operational_msg
    }
  }

  void set_coordinates(const origin_coordinate_msgs::msg::origin_coordinates::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "x: %d", msg->data.x(), msg->data.y());
    x_origin = msg.x;
    y_origin = msg.y;
    z_origin = msg.z;

    message_received = true;
  }  

  //ロボットアームの動作生成
  void move_arm(rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr client, std::vector<float> point) const {
    if(!message_received) {
      RCLCPP_WARN(this->get_logger(),"メッセージ受信なし。スキップします");
      return;
    }

    auto request = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();
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

  void move_to_harvest () {
    for (int i = 0; i < installation_point_list.size(); i++) {

      move_arm(arm_client, installation_point_list[i]);

      for (const auto &move_point: packing_mechanism_point_list) {
        move_arm(arm_client, move_point);
      }
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
  rclcpp::spin(std::make_shared<MainHarvest>());
  rclcpp::shutdown();
  return 0;
}
