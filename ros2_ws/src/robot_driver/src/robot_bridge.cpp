#include <fcntl.h>                         // File control definitions
#include <rclcpp/rclcpp.hpp>              // ROS 2 core
#include <robot_interfaces/msg/robot_control.hpp>  // Custom RobotControl message
#include <robot_interfaces/msg/wisker_data.hpp>    // Custom WiskerData message
#include <termios.h>                      // Terminal control definitions
#include <unistd.h>                       // POSIX read/write/close
#include <chrono>
#include <stdexcept>
#include <RobotIO.h>                      // Shared packed unions for serial I/O

class MotorSubscriberNode final : public rclcpp::Node {
public:
    MotorSubscriberNode()
        : Node("motor_subscriber_node") {

        // Subscribe to motor command messages
        subscription_ = this->create_subscription<robot_interfaces::msg::RobotControl>(
            "motor_output",                 // Topic name
            10,                             // QoS queue depth
            std::bind(&MotorSubscriberNode::motor_callback, this, std::placeholders::_1)
        );

        // Publish whisker sensor messages
        publisher_ = this->create_publisher<robot_interfaces::msg::WiskerData>(
            "wisker_data",
            10
        );

        // Open serial port to microcontroller
        tty_fd_ = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK);  // Non-blocking read/write
        if (tty_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            throw std::runtime_error("Failed to open serial port.");
        }

        // Setup serial port configuration
        termios tio{};
        tcgetattr(tty_fd_, &tio);                   // Get current settings
        tio.c_cflag &= ~CSIZE;                      // Clear data size bits
        tio.c_cflag |= CS8 | CREAD | CLOCAL;        // 8-bit chars, enable receiver, ignore modem control lines
        tio.c_cflag &= ~PARENB;                     // No parity
        tio.c_cflag &= ~CSTOPB;                     // One stop bit
        tio.c_lflag = 0;                            // Raw input
        tio.c_iflag = IGNPAR;                       // Ignore parity errors
        tio.c_oflag = 0;                            // Raw output
        cfsetispeed(&tio, B115200);                 // Input baud rate
        cfsetospeed(&tio, B115200);                 // Output baud rate
        tcflush(tty_fd_, TCIFLUSH);                 // Flush input buffer
        tcsetattr(tty_fd_, TCSAFLUSH, &tio);        // Apply settings

        // Run periodic task every 200ms to write motor control & read whiskers
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            [this] {
                write_control_data();    // Send last motor command to MCU
                read_wisker_data();      // Read latest whisker state from MCU
            });
    }

    ~MotorSubscriberNode() override {
        if (tty_fd_ >= 0) close(tty_fd_);  // Cleanly close serial port
    }

private:
    void motor_callback(const robot_interfaces::msg::RobotControl::SharedPtr msg) {
        last_msg_ = *msg;                   // Save most recent motor command
        new_control_available_ = true;     // Set flag to trigger write on next timer tick
    }

    void write_control_data() {
        if (!new_control_available_) return;  // Skip if nothing new to send

        RobotControlUnion ctrl{};  // Packed C-style struct
        ctrl.robot_control.left_motor_speed = last_msg_.left_motor;
        ctrl.robot_control.right_motor_speed = last_msg_.right_motor;
        ctrl.robot_control.left_motor_dir = last_msg_.left_dir;
        ctrl.robot_control.right_motor_dir = last_msg_.right_dir;
        ctrl.robot_control.dance_mode = last_msg_.dance_mode;

        uint8_t sync = SYNC_BYTE;                      // Custom sync byte for framing
        write(tty_fd_, &sync, 1);                      // Write sync byte
        write(tty_fd_, &ctrl.data, sizeof(RobotControl));  // Write raw binary struct

        new_control_available_ = false;                // Reset flag after sending
    }

    void read_wisker_data() {
        uint8_t byte;
        // Keep reading one byte at a time until we find the sync byte
        while (read(tty_fd_, &byte, 1) == 1) {
            if (byte == SYNC_BYTE) break;              // Found start of whisker packet
        }

        WiskerDataUnion data{};
        ssize_t r = read(tty_fd_, &data.data, sizeof(WiskerData));  // Read full struct

        if (r == sizeof(WiskerData)) {
            // Construct and publish ROS message from unpacked data
            robot_interfaces::msg::WiskerData msg;
            msg.w1 = data.wisker_data.w1;
            msg.w2 = data.wisker_data.w2;
            msg.w3 = data.wisker_data.w3;
            msg.w4 = data.wisker_data.w4;
            msg.w5 = data.wisker_data.w5;
            msg.w6 = data.wisker_data.w6;
            msg.wisker_count = data.wisker_data.wisker_count;

            publisher_->publish(msg);  // Push data to ROS network

            RCLCPP_INFO(this->get_logger(), "Published WiskerData: [%d %d %d %d %d %d %d]",
                        msg.w1, msg.w2, msg.w3, msg.w4, msg.w5, msg.w6, msg.wisker_count);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read full WiskerData. Got %zd bytes", r);
        }
    }

    robot_interfaces::msg::RobotControl last_msg_;  // Last received motor command
    bool new_control_available_ = false;            // Flag to control sending

    rclcpp::Subscription<robot_interfaces::msg::RobotControl>::SharedPtr subscription_;  // Motor command input
    rclcpp::Publisher<robot_interfaces::msg::WiskerData>::SharedPtr publisher_;          // Whisker data output
    rclcpp::TimerBase::SharedPtr timer_;            // Loop timer for I/O
    int tty_fd_;                                    // File descriptor for serial device
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                         // Init ROS 2 context
    rclcpp::spin(std::make_shared<MotorSubscriberNode>());  // Start node loop
    rclcpp::shutdown();                               // Shutdown cleanly
    return 0;
}
