#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <xarm_msgs/srv/move_cartesian.hpp>
#include <xarm_msgs/srv/set_digital_io.hpp>

#include "origin_coordinate_msgs/msg/origin_coordinates.hpp"

#include <unistd.h>
#include <math.h> // M_PI で円周率

using namespace std::chrono_literals;
using std::placeholders::_1;

class MainHarvest : public rclcpp::Node {
 public:
  MainHarvest() : Node("main_harvest", rclcpp::NodeOptions()),message_received(false){
    arm_client = this->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");
    end_effector_client = this-> create_client<xarm_msgs::srv::SetDigitalIO>("/xarm/set_tgpio_digital");

    //move_arm(arm_client, {300, 0, 250, 3.14, 0, 0});
    //auto_open_close(end_effector_client, false); //電源を入れると勝手に開くので閉じさせる
    //auto_open_close(end_effector_client, true);

    //timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&MainHarvest::timer_callback, this));


    coordinates_subscription = this->create_subscription<origin_coordinate_msgs::msg::OriginCoordinates>(
      "origin_points_topic", 10, std::bind(&MainHarvest::set_coordinates, this, _1)
    );
    
    std::cout << "main harvest node no recieve msg, waiting" << std::endl;
  }
  
 private:
  float x_origin;
  float y_origin;
  float z_origin;

  float cut_x_position = 106;
  float cut_y_position = -370;

  rclcpp::Subscription<origin_coordinate_msgs::msg::OriginCoordinates>::SharedPtr coordinates_subscription;

  rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr arm_client;
  rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr end_effector_client;

  bool message_received;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::vector<float>> origin_point;
  std::vector<std::vector<float>> installation_point_list;
  const std::vector<std::vector<float>> packing_mechanism_point_list = {
    {300, 0, 250, 3.14, 0, 0},
    {300, 0, 415, 3.14, 0, 0},
    {300, 0, 415, 3.14, 0, -M_PI / 2},
    {106, 0, 415, 3.14, 0, -M_PI /2},
    {106, -354, 415, 3.14, 0, -M_PI /2}, //根切設置位置前
    {106, -370, 415, 3.14, 0, -M_PI /2}, //根きり設置位置,アーム開く,終わるまで待機
    {106, -370, 412, 3.14, 0, -M_PI / 2}, 
    {106, -370, 425, 3.14, 0, -M_PI /2}, //持ち上げ
    {106, -354, 425, 3.14, 0, -M_PI /2}, //レタス持って戻る、容器片付け動作は後日追加
    //{106, -354, 425, 3.14, 0, -M_PI /2},
    {106, 0, 425, 3.14, 0, -M_PI /2},
    {106, 0, 425, 3.14, 0, 0},
    {300, 0, 250, 3.14, 0, 0}
  }; 

  //void timer_callback() {
    //return;
  //}

  

  void set_coordinates(const origin_coordinate_msgs::msg::OriginCoordinates::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "point msgs are recieved");
    x_origin = msg->x;
    y_origin = msg->y;
    z_origin = msg->z;

    message_received = true;

    //std::cout << x_origin << std::endl;

    origin_point.push_back({x_origin, y_origin, z_origin, 3.14, 0, 0});

    //std::cout << origin_point[0][2] << std::endl; データ受け取りはできている

    for (size_t i = 0; i < 5; i++) {
      float x, y ,z;

      z = 100.0; 
      if (i == 0) {
        x = x_origin + 20.0;
        y = y_origin - 200.0;
      } else {
        y = installation_point_list[i - 1][1] - 150.0;
      }
      std::vector<float> point = {x, y, z, 3.14, 0, 0};
      installation_point_list.push_back(point);
    }
    std::cout << "main harvest node is beginning..." << std::endl;
    //move_arm(arm_client, origin_point[0]); //move origin point befor harvest

    move_to_harvest();
    std::cout << "After move_to_harvest" << std::endl;  // 追加
  }  

  //ロボットアームの動作生成
  void move_arm(rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr client, std::vector<float> point) const {
    auto request = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();
    request->pose = point;
    request->speed = 150.0;
    request->acc = 50.0;
    request->mvtime = 0.0;

    while(!client -> wait_for_service(-1s)) {
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
    if(!message_received) {
      RCLCPP_WARN(this->get_logger(),"メッセージ受信なし。スキップします");
      return;
    }

    for (size_t i = 0; i < installation_point_list.size(); i++) {
      int count = 0; //寝切り位置でアームの開閉を判別するための変数
      float x_tmp_1 = installation_point_list[i][0] - 105.0;
      float x_2 = installation_point_list[i][0];
      //float x_tmp_2 = installation_point_list[i][0] + 50.0;
      float y_tmp = installation_point_list[i][1];

      move_arm(arm_client, {x_tmp_1, y_tmp, 110, 3.14, 0, 0});
      auto_open_close(end_effector_client, true);
      move_arm(arm_client, {x_2, y_tmp, 110, 3.14, 0, 0});
      auto_open_close(end_effector_client, false);
      move_arm(arm_client, {x_2, y_tmp, 150, 3.14, 0, 0});
      move_arm(arm_client, {x_tmp_1, y_tmp, 150, 3.14, 0, 0});

      std::cout << "Harvest is complete." << std::endl;

      for (const auto &move_point: packing_mechanism_point_list) {
        move_arm(arm_client, move_point);
        if (move_point[0] == cut_x_position && move_point[1] == cut_y_position) {
          if (count == 0) {
            auto_open_close(end_effector_client, true);
          } else if (count == 1) {
            auto_open_close(end_effector_client, false);
          }
          count++;
        }


        //move_arm(arm_client, move_point);
      }
      std::cout << "1 loop is finished" << std::endl;
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
    sleep(3);
  }
};

int main (int argc, char** argv){
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MainHarvest>());
  } catch (const std::exception& e) {
    std::cerr << "Exception in spin: " << e.what() << std::endl;
  }
  std::cout << "After spin" << std::endl;  // 追加
  rclcpp::shutdown();
  return 0;
}