#ifndef SOCKETCAN_HPP
#define SOCKETCAN_HPP

#include <string>
#include <linux/can.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <errno.h>

class SocketCAN {
public:
    //SocketCAN();
    SocketCAN(const std::string& interface);
    ~SocketCAN();

    bool open();
    void close();

    bool sendFrame(const struct can_frame& frame);
    bool receiveFrame(struct can_frame& frame);

    bool setNonBlocking(bool enable);

    int getSocket() const;

private:
    std::string interface_;
    int socket_;
    bool is_open_;
    bool is_non_blocking_;
};

#endif

