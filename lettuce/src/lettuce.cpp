#include <rclcpp/rclcpp.hpp>
#include <xarm_msgs/srv/move_cartesian.hpp>

using namespace std::chrono_literals;

class SampleArmController : public rclcpp::Node {
 public:
  SampleArmController() : Node("sample_arm_controller", rclcpp::NodeOptions()){

    std::cout << "INIT" << std::endl;

    const std::vector<std::vector<float>> points = {
      {200, 0, 300, 3.14, 0, 0},
      {100, 385, 108, 3.14, 0, -0.05},
      {235, 385, 108, 3.14, 0, -0.05},
      {235, 385, 98, 3.14, 0, -0.05},
      {235, 385, 98, 3.14, 0, -0.04},
      {235, 385, 200, 3.14, 0, -0.05},
      {0, 510, 130, 3.14, 0, -1.57},
      {0, 510, 75, 3.14, 0, -1.57},
      {0, 425, 75, 3.14, 0, -1.57},
      {0, 425, 110, 3.14, 0, -1.57},
      {-200, 300, 150, 3.14, 0, -1.57},
      {-200, 300, 150, 3.14, 0, -1.57},
      {200, 200, 300, 3.14, 0, 0},
      {200, 0, 300, 3.14, 0, -1.57},
      {250,450,130, 3.14, 0, -1.57},
      {375, 470, 108, 3.14, 0, -1.57},
      {375,350,108, 3.14, 0, -1.57},
      {375,350,98, 3.14, 0, -1.57},
      {375,350,98, 3.14, 0, -1.56},
      {375,350,200, 3.14, 0, -1.57},
      {0, 510, 130, 3.14, 0, -1.57},
      {0, 510, 75, 3.14, 0, -1.57},
      {0, 425, 75, 3.14, 0, -1.57},
      {0, 425, 110, 3.14, 0, -1.57},
      {-200, 300, 150, 3.14, 0, -1.57},
      {-200, 300, 150, 3.14, 0, -1.57},
      {200, 200, 300, 3.14, 0, 0},
      {200, 0, 300, 3.14, 0, -1.57},
      {250,300,130, 3.14, 0, -1.57},
      {360, 320, 108, 3.14, 0, -1.57},
      {360,200,108, 3.14, 0, -1.57},
      {360,200,98, 3.14, 0, -1.57},
      {360,200,98, 3.14, 0, -1.56},
      {360,200,200, 3.14, 0, -1.57},
      {0, 510, 130, 3.14, 0, -1.57},
      {0, 510, 75, 3.14, 0, -1.57},
      {0, 425, 75, 3.14, 0, -1.57},
      {0, 425, 110, 3.14, 0, -1.57},
      {-200, 300, 150, 3.14, 0, -1.57},
      {-200, 300, 150, 3.14, 0, -1.57},
      {200, 200, 300, 3.14, 0, 0}
      };

    client_ = this->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");

    std::cout << "create request" << std::endl;
    // create request
    for (const auto &point : points){

      std::cout << "loop" << std::endl;

      auto request = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();

      request->pose = point; 

      request->speed = 150.;
      request->acc = 50.;
      request->mvtime = 0.;

      while(!client_->wait_for_service(1s)){
        std::cout << "while" << std::endl;
        if(!rclcpp::ok()){
          RCLCPP_ERROR(this->get_logger(), 
                       "Interrupted while waiting for the service. Exiting. : %s",
                       client_->get_service_name());
          rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), 
                       "Service not vailable. waiting again ..  : %s",
                       client_->get_service_name());
      }

      //wait for result
      auto future = client_->async_send_request(request);
      auto status = future.wait_for(3.5s);
      
      if (status == std::future_status::ready){
        RCLCPP_INFO(this->get_logger(),
                  "Response: %s",
                  future.get()->message.c_str());          
      }

    }

  }
  
 private:
  rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr client_;
};

int main (int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleArmController>());
  rclcpp::shutdown();
  return 0;
}
