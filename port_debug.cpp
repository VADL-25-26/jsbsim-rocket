#include <asio.hpp>
#include <iostream>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <memory>
#include <cmath>


uint16_t vn_crc16(const uint8_t* data, size_t length)
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


void build_packet(uint8_t* packet,
                float yaw, float pitch, float roll,
                float ax, float ay, float az,
                float pressure)
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

    uint16_t crc = vn_crc16(&packet[1], 33);

    packet[34] = (crc >> 8) & 0xFF;
    packet[35] = crc & 0xFF;
}

class SerialLink {
public:
    SerialLink(const std::string& dev, unsigned baud)
        : io_(),
        work_(asio::make_work_guard(io_)),
          port_(io_, dev),
          running_(true)
    {
        port_.set_option(asio::serial_port_base::baud_rate(baud));
        port_.set_option(asio::serial_port_base::character_size(8));
        port_.set_option(asio::serial_port_base::parity(
            asio::serial_port_base::parity::none));
        port_.set_option(asio::serial_port_base::stop_bits(
            asio::serial_port_base::stop_bits::one));
        port_.set_option(asio::serial_port_base::flow_control(
            asio::serial_port_base::flow_control::none));

        start_rx();

        io_thread_ = std::thread([this]() {
            std::cout << "[IO] Starting IO thread" << std::endl;
            io_.run();
            std::cout << "[IO] IO thread exiting" << std::endl;
        });

        asio::post(io_, [] {
            std::cout << "[IO] post() executed\n";
        });
    }

    ~SerialLink() {
        running_ = false;
        asio::post(io_, [this]() {
            port_.cancel();
        });
        work_.reset();
        io_.stop();
        if (io_thread_.joinable())
            io_thread_.join();
    }

    void send(const void* data, size_t len) {
    auto buf = std::make_shared<std::vector<uint8_t>>(
        (uint8_t*)data, (uint8_t*)data + len
    );

    asio::post(io_, [this, buf]() {
        asio::async_write(port_, asio::buffer(*buf),
            [buf](const asio::error_code& ec, size_t) {
                if (ec) {
                    std::cerr << "[TX] write error: " << ec.message() << "\n";
                }
            });
    });
}


private:
    void start_rx() {
        port_.async_read_some(
            asio::buffer(rx_buf_.data(), rx_buf_.size()),
            [this](const asio::error_code& ec, size_t n) {
                if (!ec && running_) {
                    on_rx(n);
                    start_rx();
                }
            });
    }

    void on_rx(size_t n) {
        for (size_t i = 0; i < n; ++i) {
            std::cout << "[RX] Received byte: 0x" 
                      << std::hex << std::setw(2) << std::setfill('0') 
                      << (static_cast<int>(rx_buf_[i]) & 0xFF) << std::dec << "\n";
        }
    }

    asio::io_context io_;
    asio::serial_port port_;
    std::thread io_thread_;
    std::atomic<bool> running_;
    std::array<char, 256> rx_buf_;
    asio::executor_work_guard<asio::io_context::executor_type> work_;
};

int main(int argc, char* argv[]) {
    SerialLink stm32("/dev/ttyACM0", 115200);
    uint8_t output_packet[36];
    while (true) {
        build_packet(
            output_packet,
            0.0f, 0.0f, 0.0f,   // yaw, pitch, roll
            0.0f, 0.0f, 0.0f,      // ax, ay, az
            101.325f               // pressure
        );
        stm32.send(output_packet, 36);
    }

    return 0;
}
