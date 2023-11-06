#include <rclcpp/rclcpp.hpp>
#include <xarm_msgs/srv/move_cartesian.hpp>

using namespace std::chrono_literals;

class SetStartingPoint : public rclcpp::Node {
 public:
  SetStartingPoint() : Node("set_starting_point", rclcpp::NodeOptions()){

    client_ = this->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");

    std::cout << "set_starting_point node is beginning..." << std::endl;
    // create request
    auto request = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();
    request-> pose = {300, 0, 250, 3.14, 0, 0};
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
  
 private:
  rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr client_;
};

int main (int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetStartingPoint>());
  rclcpp::shutdown();
  return 0;
}
