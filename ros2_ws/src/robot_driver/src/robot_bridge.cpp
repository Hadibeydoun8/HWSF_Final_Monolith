#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/robot_control.hpp>

class MotorSubscriberNode : public rclcpp::Node {
public:
    MotorSubscriberNode()
        : Node("motor_subscriber_node") {
        subscription_ = this->create_subscription<robot_interfaces::msg::RobotControl>(
            "motor_output",
            10,
            std::bind(&MotorSubscriberNode::motor_callback, this, std::placeholders::_1)
        );
    }

private:
    void motor_callback(const robot_interfaces::msg::RobotControl::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(),
            "Received Motor Command: L=%d (%s) R=%d (%s) dance_mode=%s",
            msg->left_motor,
            msg->left_dir ? "FWD" : "REV",
            msg->right_motor,
            msg->right_dir ? "FWD" : "REV",
            msg->dance_mode ? "true" : "false");
    }

    rclcpp::Subscription<robot_interfaces::msg::RobotControl>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
