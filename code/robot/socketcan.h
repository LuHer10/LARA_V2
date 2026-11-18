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
#include <functional>
#include <mutex>

class SocketCAN {
public:
    //SocketCAN();
    SocketCAN(const std::string& interface);
    ~SocketCAN();

    bool open();
    void close();

    bool sendFrame(const struct can_frame& frame);
    bool receiveFrame(struct can_frame& frame);

    // Send `request` and wait up to `timeout_ms` for a frame accepted by
    // `match`. Holds the bus lock for the whole exchange so concurrent
    // callers (e.g. arm control thread vs. main thread) cannot steal each
    // other's replies. Stale frames queued before the request are discarded.
    // Returns false on send failure or timeout.
    bool request(const struct can_frame& request, struct can_frame& reply,
                 const std::function<bool(const struct can_frame&)>& match,
                 int timeout_ms = 20);

    bool isOpen() const { return is_open_; }

    bool setNonBlocking(bool enable);

    int getSocket() const;

private:
    std::string interface_;
    int socket_;
    bool is_open_;
    bool is_non_blocking_;
    std::mutex bus_mtx_;

    bool writeFrameUnlocked(const struct can_frame& frame);
    void drainUnlocked();
};

#endif

