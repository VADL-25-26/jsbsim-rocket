#include "racs-adapter.h"


uint16_t RACSAdapter::vnCRC16(const uint8_t* data, size_t length)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc = (crc >> 8) | (crc << 8);
        crc ^= data[i];
        crc ^= (crc & 0xFF) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xFF) << 5;
    }
    return crc;
}



void RACSAdapter::buildPacket(uint8_t* packet,
                float yaw, float pitch, float roll,
                float ax, float ay, float az,
                float pressure,
                float gx, float gy, float gz)
{
    packet[0] = 0xFA;
    packet[1] = 0x01;   // msg id
    packet[2] = 0x05;   // group flags
    packet[3] = 0x00;
    packet[4] = 0x00;
    packet[5] = 0x00;

    std::memcpy(&packet[6],  &yaw,      4);
    std::memcpy(&packet[10],  &pitch,    4);
    std::memcpy(&packet[14], &roll,     4);
    std::memcpy(&packet[18], &ax,       4);
    std::memcpy(&packet[22], &ay,       4);
    std::memcpy(&packet[26], &az,       4);
    std::memcpy(&packet[30], &pressure, 4);
    std::memcpy(&packet[34], &gx,       4);
    std::memcpy(&packet[38], &gy,       4);
    std::memcpy(&packet[42], &gz,       4);

    uint16_t crc = vn_crc16(&packet[1], PACKET_LEN - 3);

    packet[46] = (crc >> 8) & 0xFF;
    packet[47] = crc & 0xFF;
}