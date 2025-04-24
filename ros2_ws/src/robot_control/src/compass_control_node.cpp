#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/robot_control.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <cmath>

class MotorPublisherNode : public rclcpp::Node {
public:
    MotorPublisherNode() : Node("motor_publisher_node") {
        publisher_ = this->create_publisher<robot_interfaces::msg::RobotControl>("motor_output", 10);

        tty_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (tty_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open compass serial port.");
            throw std::runtime_error("Serial open failed");
        }

        struct termios tio{};
        tcgetattr(tty_fd_, &tio);
        tio.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
        tio.c_iflag = IGNPAR;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tcflush(tty_fd_, TCIFLUSH);
        tcsetattr(tty_fd_, TCSANOW, &tio);

        const std::string cmd = "1 compass.p\r\n";
        write(tty_fd_, cmd.c_str(), cmd.size());

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this] { poll_compass(); });
    }

    ~MotorPublisherNode() override {
        if (tty_fd_ >= 0) close(tty_fd_);
    }

private:
    int tty_fd_;
    rclcpp::Publisher<robot_interfaces::msg::RobotControl>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void poll_compass() {
        char buf[256] = {};
        ssize_t len = read(tty_fd_, buf, sizeof(buf) - 1);
        if (len <= 0) return;

        std::string line(buf);
        if (line.find("C,") == std::string::npos) return;

        float pitch = 0.0f, roll = 0.0f;
        try {
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream iss(line);
            std::string tag;
            int ts;
            float yaw;
            iss >> tag >> ts >> pitch >> roll >> yaw;
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Parse error: %s", line.c_str());
            return;
        }

        robot_interfaces::msg::RobotControl msg;
        constexpr int speed = 50;
        constexpr float pitch_thresh = 20.0f;
        constexpr float roll_thresh = 20.0f;

        bool forward = pitch < -pitch_thresh && std::abs(roll) < 10;
        bool backward = pitch > pitch_thresh && std::abs(roll) < 10;
        bool left = roll < -roll_thresh && std::abs(pitch) < 10;
        bool right = roll > roll_thresh && std::abs(pitch) < 10;

        if (forward) {
            msg.left_motor = msg.right_motor = speed;
            msg.left_dir = msg.right_dir = true;
        } else if (backward) {
            msg.left_motor = msg.right_motor = speed;
            msg.left_dir = msg.right_dir = false;
        } else if (left) {
            msg.left_motor = speed;
            msg.right_motor = speed;
            msg.left_dir = false;
            msg.right_dir = true;
        } else if (right) {
            msg.left_motor = speed;
            msg.right_motor = speed;
            msg.left_dir = true;
            msg.right_dir = false;
        } else {
            msg.left_motor = msg.right_motor = 0;
        }

        msg.dance_mode = false;
        msg.rouge_mode = false;

        RCLCPP_INFO(this->get_logger(),
            "Gesture => P: %+6.2f° | R: %+6.2f° => Left: %3d (%s), Right: %3d (%s)",
            pitch, roll,
            msg.left_motor,  msg.left_dir ? "FWD" : "REV",
            msg.right_motor, msg.right_dir ? "FWD" : "REV");

        publisher_->publish(msg);
    }
};
int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);                         // Init ROS 2 context
    rclcpp::spin(std::make_shared<MotorPublisherNode>());  // Start node loop
    rclcpp::shutdown();                               // Shutdown cleanly
    return 0;
}
