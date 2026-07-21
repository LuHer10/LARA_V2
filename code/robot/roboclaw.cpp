#include "roboclaw.h"

bool RoboClaw::M1Duty(uint8_t address, uint16_t duty)
{
    uint8_t packet[6];
    packet[0] = address;
    packet[1] = M1_DUTY; // Command for M1 duty cycle
    packet[2] = (duty>>8) & 0xFF;
    packet[3] = duty & 0xFF;

    uint8_t crc = crc16(packet, 4);
    packet[4] = (crc>>8) & 0xFF;
    packet[5] = crc & 0xFF;

    write(fd, packet, 6);
    tcdrain(fd);
    uint8_t recv;    
    read(fd, &recv, 1);
    if(recv == 0xFF) return true;
    return false;

}

bool RoboClaw::M2Duty(uint8_t address, uint16_t duty)
{
    uint8_t packet[6];
    packet[0] = address;
    packet[1] = M2_DUTY; // Command for M2 duty cycle
    packet[2] = (duty>>8) & 0xFF;
    packet[3] = duty & 0xFF;

    uint8_t crc = crc16(packet, 4);
    packet[4] = (crc>>8) & 0xFF;
    packet[5] = crc & 0xFF;

    write(fd, packet, 6);
    tcdrain(fd);
    uint8_t recv;    
    read(fd, &recv, 1);
    if(recv == 0xFF) return true;
    return false;
}

bool RoboClaw::M1M2Duty(uint8_t address, uint16_t duty1, uint16_t duty2)
{
    uint8_t packet[8];
    packet[0] = address;
    packet[1] = M1_M2_DUTY; // Command for M1 and M2 duty cycle
    packet[2] = (duty1>>8) & 0xFF;
    packet[3] = duty1 & 0xFF;
    packet[4] = (duty2>>8) & 0xFF;
    packet[5] = duty2 & 0xFF;

    uint16_t crc = crc16(packet, 6);
    packet[6] = (crc>>8) & 0xFF;
    packet[7] = crc & 0xFF;

    write(fd, packet, 8);
    tcdrain(fd);
    uint8_t recv;    
    read(fd, &recv, 1);
    if(recv == 0xFF) return true;
    return false;
}

bool RoboClaw::M1Speed(uint8_t address, uint32_t speed)
{
    uint8_t packet[8];
    packet[0] = address;
    packet[1] = M1_SPEED; // Command for M1 speed
    packet[2] = (speed>>24) & 0xFF;
    packet[3] = (speed>>16) & 0xFF;
    packet[4] = (speed>>8) & 0xFF;
    packet[5] = speed & 0xFF;

    uint16_t crc = crc16(packet, 6);
    packet[6] = (crc>>8) & 0xFF;
    packet[7] = crc & 0xFF;

    write(fd, packet, 8);
    tcdrain(fd);
    uint8_t recv;    
    read(fd, &recv, 1);
    if(recv == 0xFF) return true;
    return false;
}

bool RoboClaw::M2Speed(uint8_t address, uint32_t speed)
{
    uint8_t packet[8];
    packet[0] = address;
    packet[1] = M2_SPEED; // Command for M2 speed
    packet[2] = (speed>>24) & 0xFF;
    packet[3] = (speed>>16) & 0xFF;
    packet[4] = (speed>>8) & 0xFF;
    packet[5] = speed & 0xFF;

    uint16_t crc = crc16(packet, 6);
    packet[6] = (crc>>8) & 0xFF;
    packet[7] = crc & 0xFF;

    write(fd, packet, 8);
    tcdrain(fd);
    uint8_t recv;    
    read(fd, &recv, 1);
    if(recv == 0xFF) return true;
    return false;
}

bool RoboClaw::M1M2Speed(uint8_t address, uint32_t speed1, uint32_t speed2)
{
    uint8_t packet[12];
    packet[0] = address;
    packet[1] = M1_M2_SPEED; // Command for M1 and M2 speed
    packet[2] = (speed1>>24) & 0xFF;
    packet[3] = (speed1>>16) & 0xFF;
    packet[4] = (speed1>>8) & 0xFF;
    packet[5] = speed1 & 0xFF;
    packet[6] = (speed2>>24) & 0xFF;
    packet[7] = (speed2>>16) & 0xFF;
    packet[8] = (speed2>>8) & 0xFF;
    packet[9] = speed2 & 0xFF;

    uint16_t crc = crc16(packet, 10);
    packet[10] = (crc>>8) & 0xFF;
    packet[11] = crc & 0xFF;

    write(fd, packet, 12);
    tcdrain(fd);
    uint8_t recv;    
    read(fd, &recv, 1);
    if(recv == 0xFF) return true;
    return false;
}

bool RoboClaw::allSpeed(uint32_t speed1, uint32_t speed2, uint32_t speed3, uint32_t speed4)
{
    bool recv;
    recv = M1M2Speed(FRONT_ADDRESS, speed1, speed2);
    recv &= M1M2Speed(BACK_ADDRESS, speed3, speed4);
    return recv;
}

bool RoboClaw::allDuty(uint16_t duty1, uint16_t duty2, uint16_t duty3, uint16_t duty4)
{
    bool recv;
    recv = M1M2Duty(FRONT_ADDRESS, duty1, duty2);
    recv &= M1M2Duty(BACK_ADDRESS, duty3, duty4);
    return recv;
}

bool RoboClaw::readEncoders(uint8_t address, uint32_t& encoder1, uint32_t& encoder2) {
    
    
    // Read 10 bytes: 4 bytes of encoder1, 4 bytes of encoder2, 2 CRC
    uint8_t response[10];
    readCommand(address, M1_M2_ENCODERS, response, 10);

    uint32_t encoderValue1 = ((uint32_t)response[0] << 24) |
                             ((uint32_t)response[1] << 16) |
                             ((uint32_t)response[2] << 8)  |
                             (uint32_t)response[3];

    uint32_t encoderValue2 = ((uint32_t)response[4] << 24) |
                             ((uint32_t)response[5] << 16) |
                             ((uint32_t)response[6] << 8)  |
                             (uint32_t)response[7];
                        
    encoder1 = encoderValue1;
    encoder2 = encoderValue2;

    return true;
}

bool RoboClaw::readCommand(uint8_t address, uint8_t command, uint8_t* rxData, size_t n) {
    uint8_t packet[2];
    packet[0] = address;
    packet[1] = command;

    uint16_t crc = crc16(packet, 2);

    // Send command
    if (write(fd, packet, 2) != 2) return false;
    tcdrain(fd);
    
    // Read response
    ssize_t bytesRead = read(fd, rxData, n);
    if (bytesRead != n) return false;
    
    // Verify CRC
    crc = crc16(rxData, n - 2, crc);
    uint16_t receivedCrc = (rxData[n - 2] << 8) | rxData[n - 1];
    
    return crc == receivedCrc;
}

bool RoboClaw::resetEncoders(uint8_t address)
{
    uint8_t packet[4];
    packet[0] = address;
    packet[1] = RESET_ENCODERS; // Command for resetting encoders

    uint16_t crc = crc16(packet, 2);
    packet[2] = (crc >> 8) & 0xFF;
    packet[3] = crc & 0xFF;

    write(fd, packet, 4);
    tcdrain(fd);
    uint8_t recv;
    read(fd, &recv, 1);
    if(recv == 0xFF) return true;
    return false;
}

bool RoboClaw::readSpeeds(uint8_t address, int32_t& speed1, int32_t& speed2)
{
    bool recv;
    uint8_t response[7];
    if(recv  = readCommand(address, READ_M1_SPEED, response, 7))
        speed1 = ((uint32_t)response[0] << 24) |
                 ((uint32_t)response[1] << 16) |
                 ((uint32_t)response[2] << 8)  |
                 (uint32_t)response[3] * (uint32_t)(response[4]? -1 : 1);
    if(recv &= readCommand(address, READ_M2_SPEED, response, 7))
        speed2 = ((uint32_t)response[0] << 24) |
                 ((uint32_t)response[1] << 16) |
                 ((uint32_t)response[2] << 8)  |
                 (uint32_t)response[3] * (uint32_t)(response[4]? -1 : 1);
    return recv;
}
