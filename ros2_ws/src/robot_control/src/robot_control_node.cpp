#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/robot_control.hpp>
#include <robot_interfaces/msg/wisker_data.hpp>
#include <RobotIO.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>
#include <cmath>

class MotorPublisherNode final : public rclcpp::Node {
public:
    MotorPublisherNode()
        : Node("motor_publisher_node") {

        publisher_ = this->create_publisher<robot_interfaces::msg::RobotControl>("motor_output", 10);

        // Subscribe to whisker data
        wisker_sub_ = this->create_subscription<robot_interfaces::msg::WiskerData>(
            "wisker_data", 10,
            std::bind(&MotorPublisherNode::wisker_callback, this, std::placeholders::_1)
        );

        tty_fd_ = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK);
        if (tty_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            throw std::runtime_error("Failed to open serial port.");
        }

        struct termios tio{};
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
        tcsetattr(tty_fd_, TCSANOW, &tio);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            [this] { poll_serial(); });
    }

    ~MotorPublisherNode() override {
        if (tty_fd_ >= 0) close(tty_fd_);
    }

private:
    ControllerDataUnion u_controller_d{};
    bool dance_mode_ = false;
    robot_interfaces::msg::WiskerData last_wisker_;
    rclcpp::Subscription<robot_interfaces::msg::WiskerData>::SharedPtr wisker_sub_;

    int32_t map_joystick_to_speed(int16_t val, bool &dir, bool flip) {
        constexpr int CENTER = 512;
        constexpr int DEADZONE_MIN = 500;
        constexpr int DEADZONE_MAX = 520;

        if (val >= DEADZONE_MIN && val <= DEADZONE_MAX) {
            dir = true;
            return 0;
        }

        int16_t delta = val - CENTER;
        if (flip) delta = -delta;

        dir = delta > 0;

        int16_t abs_delta = std::abs(delta);
        int16_t max_delta = std::max(CENTER - DEADZONE_MIN, 1024 - DEADZONE_MAX);
        int32_t speed = (abs_delta * 100) / max_delta;

        return std::clamp(speed, 0, 100);
    }

    void poll_serial() {
        uint8_t byte;
        if (read(tty_fd_, &byte, 1) == 1 && byte == SYNC_BYTE) {
            ssize_t r = read(tty_fd_, u_controller_d.data, sizeof(ControllerData));
            if (r == sizeof(ControllerData)) {
                const auto& input = u_controller_d.controller_data;

                robot_interfaces::msg::RobotControl msg;

                // Check if dance mode should be activated
                if (!dance_mode_ && last_wisker_.wisker_count >= 10) {
                    dance_mode_ = true;
                    RCLCPP_INFO(this->get_logger(), "DANCE MODE ACTIVATED! Wisker count = %d", last_wisker_.wisker_count);
                }

                // Check if dance mode should be deactivated via joystick button
                if (dance_mode_ && input.control) {
                    dance_mode_ = false;
                    RCLCPP_INFO(this->get_logger(), "Dance mode deactivated via control button.");
                }

                RCLCPP_INFO(this->get_logger(), "Whisker Count: %d", last_wisker_.wisker_count);


                // New logic: y_pos = forward/back, x_pos = turning
                bool forward_dir, turn_dir;
                int32_t forward = map_joystick_to_speed(input.y_pos, forward_dir, true);  // 1024 = FWD
                int32_t turn = map_joystick_to_speed(input.x_pos, turn_dir, false);       // 1024 = RIGHT

                // Mixed turning logic (opposite wheels turn)
                int left = forward;
                int right = forward;

                if (turn > 0) {
                    left += turn;
                    right -= turn;
                }

                msg.left_motor = std::clamp(std::abs(left), 0, 100);
                msg.right_motor = std::clamp(std::abs(right), 0, 100);

                msg.left_dir = left >= 0 ? forward_dir : !forward_dir;
                msg.right_dir = right >= 0 ? forward_dir : !forward_dir;

                msg.dance_mode = dance_mode_;

                publisher_->publish(msg);

                RCLCPP_INFO(this->get_logger(),
                    "Published: L=%d (%s) R=%d (%s) dance=%s",
                    msg.left_motor,
                    msg.left_dir ? "FWD" : "REV",
                    msg.right_motor,
                    msg.right_dir ? "FWD" : "REV",
                    msg.dance_mode ? "true" : "false");
            }
        }
    }

    void wisker_callback(const robot_interfaces::msg::WiskerData::SharedPtr msg) {
        last_wisker_ = *msg;
    }

    rclcpp::Publisher<robot_interfaces::msg::RobotControl>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int tty_fd_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
