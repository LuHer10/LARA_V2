#ifndef NETWORK_H
#define NETWORK_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <time.h>

#define PORT_RECEIVE 40000
#define PORT_SEND 40001
#define BUFFER_SIZE 1024
#define TIMEOUT_SECONDS 2

#define EXPECTED_PACKET_SIZE 18  //1 header + 4 floats + 1 int8
#define HEADER_BYTE 0xAB
#define SEND_PACKET_SIZE 13 //1 header + 3 floats

class Network
{
    int sockfd;
    struct sockaddr_in server_addr, client_addr, send_addr;
    socklen_t client_len = sizeof(client_addr);
    char buffer[BUFFER_SIZE];

    struct timeval last_receive_time, current_time, last_send_time;
    int timeout_printed = 0;
    int client_known = 0;

public:
    Network()
    {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        // Set non-blocking mode
        int flags = fcntl(sockfd, F_GETFL, 0);
        fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(PORT_RECEIVE);

        if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            perror("bind failed");
            close(sockfd);
            exit(EXIT_FAILURE);
        }

        printf("Robot UDP server listening on port %d\n", PORT_RECEIVE);

        gettimeofday(&last_receive_time, NULL);
        gettimeofday(&last_send_time, NULL);

    }

    ~Network()
    {
        close(sockfd);
    }

    ssize_t receive(const char *buf);
    void send(const char *buf);
    void sendOdometry(float x, float y, float ang, long dt_us  = 100000);
    void receiveJoysticks(float &x_l, float &y_l, float &x_r, float &y_r, int8_t &grip);
};

#endif //NETWORK_H