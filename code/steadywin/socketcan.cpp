#include "socketcan.h"

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <errno.h>

SocketCAN::SocketCAN(const std::string& interface)
    : interface_(interface), socket_(-1), is_open_(false), is_non_blocking_(false) {}

SocketCAN::~SocketCAN() {
    close();
}

bool SocketCAN::open() {
    struct ifreq ifr;
    struct sockaddr_can addr;

    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
        perror("Socket");
        return false;
    }

    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        return false;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        return false;
    }

    is_open_ = true;
    return true;
}

bool SocketCAN::setNonBlocking(bool enable) {
    if (!is_open_) return false;

    int flags = fcntl(socket_, F_GETFL, 0);
    if (flags < 0) return false;

    if (enable)
        flags |= O_NONBLOCK;
    else
        flags &= ~O_NONBLOCK;

    if (fcntl(socket_, F_SETFL, flags) < 0)
        return false;

    is_non_blocking_ = enable;
    return true;
}

void SocketCAN::close() {
    if (is_open_) {
        ::close(socket_);
        is_open_ = false;
    }
}

bool SocketCAN::sendFrame(const struct can_frame& frame) {
    if (!is_open_) return false;

    int nbytes = write(socket_, &frame, sizeof(frame));

    if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // Would block → not an error in non-blocking mode
            return false;
        }
        perror("write");
        return false;
    }

    return nbytes == sizeof(frame);
}

bool SocketCAN::receiveFrame(struct can_frame& frame) {
    if (!is_open_) return false;

    int nbytes = read(socket_, &frame, sizeof(frame));
    //std::cout << nbytes << "\n"; 

    if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No data available right now
            return false;
        }
        perror("read");
        return false;
    }

    return nbytes == sizeof(frame);
}

int SocketCAN::getSocket() const {
    return socket_;
}


