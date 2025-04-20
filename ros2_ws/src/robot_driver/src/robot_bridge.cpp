#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/robot_control.hpp>
#include <robot_interfaces/msg/wisker_data.hpp>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <stdexcept>
#include <RobotIO.h>

class MotorSubscriberNode final : public rclcpp::Node {
public:
    MotorSubscriberNode()
        : Node("motor_subscriber_node") {

        subscription_ = this->create_subscription<robot_interfaces::msg::RobotControl>(
            "motor_output",
            10,
            std::bind(&MotorSubscriberNode::motor_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<robot_interfaces::msg::WiskerData>(
            "wisker_data",
            10
        );

        // Serial setup
        tty_fd_ = open("/dev/ttyACM1", O_RDWR | O_NONBLOCK);
        if (tty_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            throw std::runtime_error("Failed to open serial port.");
        }

        termios tio{};
        tcgetattr(tty_fd_, &tio);
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8 | CREAD | CLOCAL;
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_lflag = 0;
        tio.c_iflag = IGNPAR;
        tio.c_oflag = 0;
        cfsetispeed(&tio, B115200);
        cfsetospeed(&tio, B115200);
        tcflush(tty_fd_, TCIFLUSH);
        tcsetattr(tty_fd_, TCSAFLUSH, &tio);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),  // <- now 200ms
            [this] {
                write_control_data();
                read_wisker_data();
            });
    }

    ~MotorSubscriberNode() override {
        if (tty_fd_ >= 0) close(tty_fd_);
    }

private:
    void motor_callback(const robot_interfaces::msg::RobotControl::SharedPtr msg) {
        last_msg_ = *msg;  // store the latest control input
        new_control_available_ = true;

        // RCLCPP_INFO(this->get_logger(),
        //     "Received Motor Command: L=%d (%s) R=%d (%s) dance_mode=%s",
        //     msg->left_motor,
        //     msg->left_dir ? "FWD" : "REV",
        //     msg->right_motor,
        //     msg->right_dir ? "FWD" : "REV",
        //     msg->dance_mode ? "true" : "false");
    }

    void write_control_data() {
        if (!new_control_available_) return;

        RobotControlUnion ctrl{};
        ctrl.robot_control.left_motor_speed = last_msg_.left_motor;
        ctrl.robot_control.right_motor_speed = last_msg_.right_motor;
        ctrl.robot_control.left_motor_dir = last_msg_.left_dir;
        ctrl.robot_control.right_motor_dir = last_msg_.right_dir;
        ctrl.robot_control.dance_mode = last_msg_.dance_mode;

        uint8_t sync = SYNC_BYTE;
        write(tty_fd_, &sync, 1);
        write(tty_fd_, &ctrl.data, sizeof(RobotControl));

        new_control_available_ = false; // reset flag after sending
    }

    void read_wisker_data() {
        uint8_t byte;
        // Sync until we see SYNC_BYTE
        while (read(tty_fd_, &byte, 1) == 1) {
            if (byte == SYNC_BYTE) {
                break;
            }
        }

        // If we found SYNC_BYTE, read the rest
        WiskerDataUnion data{};
        ssize_t r = read(tty_fd_, &data.data, sizeof(WiskerData));
        if (r == sizeof(WiskerData)) {
            robot_interfaces::msg::WiskerData msg;
            msg.w1 = data.wisker_data.w1;
            msg.w2 = data.wisker_data.w2;
            msg.w3 = data.wisker_data.w3;
            msg.w4 = data.wisker_data.w4;
            msg.w5 = data.wisker_data.w5;
            msg.w6 = data.wisker_data.w6;
            msg.wisker_count = data.wisker_data.wisker_count;

            publisher_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Published WiskerData: [%d %d %d %d %d %d %d]",
                        msg.w1, msg.w2, msg.w3, msg.w4, msg.w5, msg.w6, msg.wisker_count);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read full WiskerData. Got %zd bytes", r);
        }
    }

    robot_interfaces::msg::RobotControl last_msg_;
    bool new_control_available_ = false;

    rclcpp::Subscription<robot_interfaces::msg::RobotControl>::SharedPtr subscription_;
    rclcpp::Publisher<robot_interfaces::msg::WiskerData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int tty_fd_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
