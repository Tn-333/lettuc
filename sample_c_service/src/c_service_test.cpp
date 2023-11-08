#include "rclcpp/rclcpp.hpp"
#include "xarm_msgs/srv/plan_pose.hpp"
#include <xarm_msgs/srv/set_digital_io.hpp>

#include <chrono>
#include <memory>
#include <cstdio>

using namespace std::chrono_literals;

// 関数にクライアントを渡す
void auto_open_close(rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr client, bool open_close_flag);

void auto_open_close(rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr client, bool open_close_flag) {
    std::string show_msg = "";
    auto request = std::make_shared<xarm_msgs::srv::SetDigitalIO::Request>();
    request->ionum = 0;

    if (open_close_flag) {
        request->value = 0; // open
        show_msg = "open";
    } else {
        request->value = 1; // close
        show_msg = "close";
    }

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "サービス待機中に中断されました。終了します。");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "サービスが利用できません、再度待機中...");
    }

    std::cout << show_msg << std::endl;
    auto result = client->async_send_request(request);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("c_service_sample");
    rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr client =
        node->create_client<xarm_msgs::srv::SetDigitalIO>("/xarm/set_tgpio_digital");

    auto_open_close(client, true);
    // wait result

    rclcpp::shutdown();
    return 0;
}
