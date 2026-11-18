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
#include <poll.h>
#include <chrono>

SocketCAN::SocketCAN(const std::string& interface)
    : interface_(interface), socket_(-1), is_open_(false), is_non_blocking_(false) 
    {
        //setNonBlocking(is_non_blocking_);
    }

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

bool SocketCAN::writeFrameUnlocked(const struct can_frame& frame) {
    if (!is_open_) return false;

    int nbytes = write(socket_, &frame, sizeof(frame));

    if (nbytes < 0) {
        // EAGAIN/EWOULDBLOCK (non-blocking, tx queue full) or a real error
        return false;
    }

    return nbytes == sizeof(frame);
}

bool SocketCAN::sendFrame(const struct can_frame& frame) {
    std::lock_guard<std::mutex> lock(bus_mtx_);
    return writeFrameUnlocked(frame);
}

bool SocketCAN::receiveFrame(struct can_frame& frame) {
    std::lock_guard<std::mutex> lock(bus_mtx_);
    if (!is_open_) return false;

    int nbytes = read(socket_, &frame, sizeof(frame));
    //std::cout << nbytes << "\n"; 

    if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No data available right now
            return false;
        }
        //perror("read");
        return false;
    }

    return nbytes == sizeof(frame);
}

void SocketCAN::drainUnlocked() {
    struct can_frame junk;
    while (recv(socket_, &junk, sizeof(junk), MSG_DONTWAIT) > 0) {}
}

bool SocketCAN::request(const struct can_frame& request, struct can_frame& reply,
                        const std::function<bool(const struct can_frame&)>& match,
                        int timeout_ms) {
    using clock = std::chrono::steady_clock;
    std::lock_guard<std::mutex> lock(bus_mtx_);
    if (!is_open_) return false;

    drainUnlocked();
    if (!writeFrameUnlocked(request)) return false;

    const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
    while (true) {
        long remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                             deadline - clock::now()).count();
        if (remaining <= 0) return false;

        struct pollfd pfd{socket_, POLLIN, 0};
        int r = poll(&pfd, 1, (int)remaining);
        if (r < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (r == 0) return false; // timeout

        struct can_frame frame;
        ssize_t n = recv(socket_, &frame, sizeof(frame), MSG_DONTWAIT);
        if (n == (ssize_t)sizeof(frame) && match(frame)) {
            reply = frame;
            return true;
        }
        // otherwise: unrelated frame, keep waiting
    }
}

int SocketCAN::getSocket() const {
    return socket_;
}


