

class RACSAdapter {
public:
    void 

private:
    uint16_t vnCRC16(const uint8_t* data, size_t length);
    void buildPacket(uint8_t* packet,
                float yaw, float pitch, float roll,
                float ax, float ay, float az,
                float pressure,
                float gx, float gy, float gz);
}