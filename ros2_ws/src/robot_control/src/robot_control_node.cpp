// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/robot_control.hpp>
#include <robot_interfaces/msg/wisker_data.hpp>

// Shared struct for serial I/O
#include <RobotIO.h>

// POSIX includes for serial port configuration
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

        // Publishes joystick-derived motor commands
        publisher_ = this->create_publisher<robot_interfaces::msg::RobotControl>("motor_output", 10);

        // Subscribes to whisker sensor input to decide when to dance
        wisker_sub_ = this->create_subscription<robot_interfaces::msg::WiskerData>(
            "wisker_data", 10,
            std::bind(&MotorPublisherNode::wisker_callback, this, std::placeholders::_1)
        );

        // Setup serial connection to the joystick controller
        tty_fd_ = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK);
        if (tty_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            throw std::runtime_error("Failed to open serial port.");
        }

        // Configure serial port settings
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

        // Timer for polling joystick input at 50ms interval
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this] { poll_serial(); });
    }

    ~MotorPublisherNode() override {
        if (tty_fd_ >= 0) close(tty_fd_);
    }

private:
    ControllerDataUnion u_controller_d{};
    bool dance_mode_ = false;
    bool rouge_mode_ = false;
    robot_interfaces::msg::WiskerData last_wisker_;
    rclcpp::Subscription<robot_interfaces::msg::WiskerData>::SharedPtr wisker_sub_;

    /**
     * Convert joystick value to motor speed (0–100)
     * Also outputs direction based on sign of offset from center
     */
    static int32_t map_joystick_to_speed(const int16_t val, bool &dir, const bool flip) {
        constexpr int CENTER = 512;
        constexpr int DEADZONE_MIN = 500;
        constexpr int DEADZONE_MAX = 520;

        // Within deadzone — treat as 0 speed
        if (val >= DEADZONE_MIN && val <= DEADZONE_MAX) {
            dir = true;
            return 0;
        }

        // Flip polarity for joystick inversion
        int16_t delta = val - CENTER;
        if (flip) delta = -delta;

        dir = delta > 0; // True = forward

        // Normalize delta to 0–100 range
        int16_t abs_delta = std::abs(delta);
        int16_t max_delta = std::max(CENTER - DEADZONE_MIN, 1024 - DEADZONE_MAX);
        int32_t speed = (abs_delta * 100) / max_delta;

        return std::clamp(speed, 0, 100);
    }

    void poll_serial() {
        uint8_t byte;

        // Wait for sync byte to ensure alignment
        if (read(tty_fd_, &byte, 1) == 1 && byte == SYNC_BYTE) {
            ssize_t r = read(tty_fd_, u_controller_d.data, sizeof(ControllerData));
            if (r == sizeof(ControllerData)) {
                const auto& input = u_controller_d.controller_data;

                robot_interfaces::msg::RobotControl msg;

                // Enable dance mode when enough whiskers have triggered
                if (last_wisker_.wisker_count >= 10) {
                    rouge_mode_ = true;
                    RCLCPP_INFO(this->get_logger(), "DANCE MODE ACTIVATED! Wisker count = %d", last_wisker_.wisker_count);
                }

                if (input.left)
                {
                    dance_mode_ = true;
                }

                // Manual override to disable dance mode and rouge mode
                if (input.control) {
                    dance_mode_ = false;
                    rouge_mode_ = false;
                    RCLCPP_INFO(this->get_logger(), "Dance mode deactivated via control button.");
                }

                // Debug whisker feedback
                RCLCPP_INFO(this->get_logger(), "Whisker Count: %d", last_wisker_.wisker_count);

                // Y controls forward/backward, X controls turning
                bool forward_dir, turn_dir;
                int32_t forward = map_joystick_to_speed(input.y_pos, forward_dir, true);  // Up on joystick is forward
                int32_t turn = map_joystick_to_speed(input.x_pos, turn_dir, false);       // Right on joystick = right turn

                // Default both wheels go forward
                int left = forward;
                int right = forward;

                // Apply turn as differential adjustment
                if (turn > 0) {
                    left += turn;
                    right -= turn;
                }

                // Clamp speed and determine direction
                msg.left_motor = std::clamp(std::abs(left), 0, 100);
                msg.right_motor = std::clamp(std::abs(right), 0, 100);

                msg.left_dir = left >= 0 ? forward_dir : !forward_dir;
                msg.right_dir = right >= 0 ? forward_dir : !forward_dir;

                msg.dance_mode = dance_mode_;

                msg.rouge_mode = rouge_mode_;

                // Publish the final command
                publisher_->publish(msg);

                // Note: RCLCPP_INFO printf is incorrect (e.g., using * for bool) — fix if needed
                RCLCPP_INFO(this->get_logger(),
                    "Published MotorCmd | L: %d (%s) R: %d (%s) | dance=%s",
                    msg.left_motor,
                    msg.left_dir ? "FWD" : "REV",
                    msg.right_motor,
                    msg.right_dir ? "FWD" : "REV",
                    msg.dance_mode ? "true" : "false");
            }
        }
    }

    void wisker_callback(const robot_interfaces::msg::WiskerData::SharedPtr msg) {
        // Save the latest whisker sensor state
        last_wisker_ = *msg;
    }

    rclcpp::Publisher<robot_interfaces::msg::RobotControl>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int tty_fd_;
};

int main(const int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
