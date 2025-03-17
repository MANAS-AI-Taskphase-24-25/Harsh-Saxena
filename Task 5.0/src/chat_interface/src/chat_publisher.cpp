#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>

class ChatPublisher : public rclcpp::Node {
public:
    ChatPublisher() : Node("chat_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("chat_topic", 10);
        RCLCPP_INFO(this->get_logger(), "Talker: ");
        send_messages();
    }

private:
    void send_messages() {
        while (rclcpp::ok()) {
            std::string user_input;
            std::cout << "You: ";
            std::getline(std::cin, user_input);

            auto msg = std_msgs::msg::String();
            msg.data = user_input;
            publisher_->publish(msg);
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChatPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

