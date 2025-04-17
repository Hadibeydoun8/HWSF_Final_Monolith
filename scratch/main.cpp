#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

// #include "/home/ws/Shared/Include/WiskerData.h"
struct __attribute__ ((packed)) ControllerData {
    int16_t x_pos;
    int16_t y_pos;

    bool control;

    bool up;
    bool down;
    bool left;
    bool right;
};

int open_serial_tcp(const char* hostname, int port) {
    std::cout << "[DEBUG] Resolving: " << hostname << ":" << port << "\n";

    struct addrinfo hints{}, *res;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    int err = getaddrinfo(hostname, std::to_string(port).c_str(), &hints, &res);
    if (err != 0) {
        std::cerr << "[ERROR] getaddrinfo: " << gai_strerror(err) << "\n";
        return -1;
    }

    int sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sock < 0) {
        perror("socket");
        freeaddrinfo(res);
        return -1;
    }

    if (connect(sock, res->ai_addr, res->ai_addrlen) < 0) {
        perror("connect");
        close(sock);
        freeaddrinfo(res);
        return -1;
    }

    freeaddrinfo(res);
    std::cout << "[DEBUG] Connected successfully!\n";
    return sock;
}

int main() {
    const char* hostname = "host.docker.internal";
    int port = 5001;

    int sock_fd = open_serial_tcp(hostname, port);
    if (sock_fd < 0) return 1;

    std::cout << "Connected. Reading WiskerData structs...\n";

    while (true) {
        ControllerData data{};
        ssize_t n = read(sock_fd, &data, 1);
        std::cout << n << " bytes read\n";
        if (true) {
        std::cout << "x_pos: " << data.x_pos
          << ", y_pos: " << data.y_pos
          << ", control: " << data.control
          << ", up: " << data.up
          << ", down: " << data.down
          << ", left: " << data.left
          << ", right: " << data.right
          << std::endl;


        } else {
            usleep(1000); // throttle if nothing yet
        }
    }

    close(sock_fd);
    return 0;
}
x``