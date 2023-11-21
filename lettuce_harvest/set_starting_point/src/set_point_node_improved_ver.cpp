#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <xarm_msgs/srv/move_cartesian.hpp>
#include <xarm_msgs/srv/set_digital_io.hpp>

#include "origin_coordinate_msgs/msg/origin_coordinates.hpp"

#include <unistd.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SetStartingPoint : public rclcpp::Node {
 public:
  SetStartingPoint() : Node("set_starting_point", rclcpp::NodeOptions()){
    arm_client = this->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");
    end_effector_client = this-> create_client<xarm_msgs::srv::SetDigitalIO>("/xarm/set_tgpio_digital");

    //auto_open_close(end_effector_client, false); //電源を入れると勝手に開くので閉じさせる

    operational_subscription = this->create_subscription<std_msgs::msg::String>(
      "move_origin_topic", 10, std::bind(&SetStartingPoint::topic_callback, this, _1)
    );

    origin_coordinate_publisher = this->create_publisher<origin_coordinate_msgs::msg::OriginCoordinates>("origin_points_topic", 10);
    feedback_publisher = this -> create_publisher<std_msgs::msg::Bool>("feedback_topic", 10);


    //create client


    

    std::cout << "set_starting_point node is beginning..." << std::endl;
    message_received = false;
  
  }
  
 private:
  mutable float x_origin;
  mutable float y_origin;
  mutable float z_origin;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operational_subscription;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr feedback_publisher;
  rclcpp::Publisher<origin_coordinate_msgs::msg::OriginCoordinates>::SharedPtr origin_coordinate_publisher;


  rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr arm_client;
  rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr end_effector_client;

  std::vector<float> point = {300, 0, 250, 3.14, 0, 0};

  bool message_received;


  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: %s", msg->data.c_str());
    message_received = true;
    move_robot_arm(arm_client, msg->data.c_str());
  }  

  void publish_points() const {
    auto point = origin_coordinate_msgs::msg::OriginCoordinates();
    point.x = SetStartingPoint::x_origin;
    point.y = SetStartingPoint::y_origin;
    point.z = SetStartingPoint::z_origin;


    RCLCPP_INFO(this->get_logger(), "publish:x_origin = '%f'",point.x);
    RCLCPP_INFO(this->get_logger(), "publish:y_origin = '%f'",point.y);
    origin_coordinate_publisher->publish(point);
  }

  void give_feedback() const {
    sleep(3);
    auto message = std_msgs::msg::Bool();
    message.data = true;
    RCLCPP_INFO(this -> get_logger(), "Arm movement is complete");
    feedback_publisher -> publish(message);
  }

  //ロボットアームの動作生成
  void move_robot_arm(rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr client, std::string operational_msg) {
    auto request = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();

    if (operational_msg == "x_up") {
      point[0] += 20;
    } else if (operational_msg == "x_down") {
      point[0] -= 20;
    } else if (operational_msg == "y_up") {
      point[1] += 20;
    } else if (operational_msg == "y_down") {
      point[1] -= 20;
    } else if (operational_msg == "set_complete") {
      x_origin = point[0];
      y_origin = point[1];
      z_origin = point[2];

      publish_points();
      return;
    }

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
    give_feedback();
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
