#ifndef ROBOCLAW_H
#define ROBOCLAW_H

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>

#define FRONT_ADDRESS 0x80
#define BACK_ADDRESS  0x81
#define DEFAULT_PORT "/dev/ttyAMA0"
#define DEFAULT_BAUDS B38400


enum commands:uint8_t{
    READ_M1_SPEED = 18,
    READ_M2_SPEED = 19,
    RESET_ENCODERS = 20,
    M1_DUTY = 32,
    M2_DUTY = 33,
    M1_M2_DUTY = 34,
    M1_SPEED = 35,  
    M2_SPEED = 36,
    M1_M2_SPEED = 37,
    M1_M2_ENCODERS = 78
};

class RoboClaw
{
private:
    int baudRate;
    const char* serialPort;

    int fd;

    uint16_t crc16(const uint8_t* data, size_t length, uint16_t init = 0) {
        uint16_t crc = init;
        for(size_t i = 0; i < length; ++i) {
            crc ^= ((uint16_t)data[i]) << 8;
            for(int j = 0; j < 8; ++j) {
                if(crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
                else
                    crc <<= 1;
            }
        }
        return crc;
    }


public:
    RoboClaw(const char* _serialPort =  DEFAULT_PORT, int _baudRate = DEFAULT_BAUDS)
    {
        serialPort = _serialPort;
        baudRate = _baudRate;
        fd = open(serialPort, O_RDWR | O_NOCTTY);
        if (fd == -1)
            perror("openSerialPort: Unable to open port");

        struct termios options{};
        tcgetattr(fd, &options);
        cfsetispeed(&options, _baudRate);
        cfsetospeed(&options, _baudRate);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CRTSCTS;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;
        tcsetattr(fd, TCSANOW, &options);

    }

    ~RoboClaw(){close(fd);}

    bool M1Duty(uint8_t address, uint16_t duty);
    bool M2Duty(uint8_t address, uint16_t duty);
    bool M1M2Duty(uint8_t address, uint16_t duty1, uint16_t duty2);

    bool M1Speed(uint8_t address, uint32_t speed);
    bool M2Speed(uint8_t address, uint32_t speed);
    bool M1M2Speed(uint8_t address, uint32_t speed1, uint32_t speed2);
    
    bool allSpeed(uint32_t speed1, uint32_t speed2, uint32_t speed3, uint32_t speed4);
    bool allDuty(uint16_t duty1, uint16_t duty2, uint16_t duty3, uint16_t duty4);

    bool readCommand(uint8_t address, uint8_t command, uint8_t* rxData, size_t n);

    bool readEncoders(uint8_t address, uint32_t& encoder1, uint32_t& encoder2);
    bool resetEncoders(uint8_t address);

    bool readSpeeds(uint8_t address, int32_t& speed1, int32_t& speed2);

};

#endif //ROBOCLAW_H
