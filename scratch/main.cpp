#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>

struct __attribute__((packed)) ControllerData {
    int16_t x_pos;
    int16_t y_pos;

    bool control;

    bool up;
    bool down;
    bool left;
    bool right;
};

constexpr uint8_t SYNC_BYTE = 0xAA;

int main() {
    struct termios tio;
    struct termios stdio;
    struct termios old_stdio;

    int tty_fd;
    unsigned char c = 'D';

    // Terminal config
    tcgetattr(STDOUT_FILENO, &old_stdio);
    memset(&stdio, 0, sizeof(stdio));
    stdio.c_lflag |= ECHO;
    stdio.c_iflag |= ICRNL;
    stdio.c_oflag |= (OPOST | ONLCR);
    tcsetattr(STDOUT_FILENO, TCSAFLUSH, &stdio);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    // Serial config
    tty_fd = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK);
    if (tty_fd < 0) {
        perror("open");
        return 1;
    }

    tcgetattr(tty_fd, &tio);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8 | CREAD | CLOCAL;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_lflag = 0;
    cfsetospeed(&tio, B115200);
    cfsetispeed(&tio, B115200);
    tcsetattr(tty_fd, TCSAFLUSH, &tio);

    std::cout << "Serial port opened. Waiting for sync byte...\n";

    while (true) {
        uint8_t byte;
        ssize_t n = read(tty_fd, &byte, 1);
        if (n == 1) {
            if (byte == SYNC_BYTE) {
                ControllerData data;
                size_t received = 0;
                uint8_t* ptr = reinterpret_cast<uint8_t*>(&data);

                while (received < sizeof(ControllerData)) {
                    ssize_t chunk = read(tty_fd, ptr + received, sizeof(ControllerData) - received);
                    if (chunk > 0) {
                        received += chunk;
                    }
                }

                std::cout << "Controller: { "
                          << "x: " << data.x_pos
                          << ", y: " << data.y_pos
                          << ", ctrl: " << data.control
                          << ", ↑: " << data.up
                          << ", ↓: " << data.down
                          << ", ←: " << data.left
                          << ", →: " << data.right
                          << " }\n";
            }
        } else {
            usleep(100); // idle wait
        }
    }

    close(tty_fd);
    tcsetattr(STDOUT_FILENO, TCSANOW, &old_stdio);
    return 0;
}
