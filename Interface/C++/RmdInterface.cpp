#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <boost/asio.hpp> // For serial port operations
#include <fmt/core.h> // For formatting strings, similar to f-strings in Python

class RmdMotor {
public:
    RmdMotor(int motor_id, boost::asio::serial_port& serial_port)
    : id(fmt::format("{:02X}", motor_id)), serial(serial_port) {}

    std::vector<int> write(const std::vector<int>& send_buffer, int buffer_size = 13) {
        auto crc_buffer = calculate_crc(send_buffer);
        boost::asio::write(serial, boost::asio::buffer(crc_buffer));

        std::vector<int> dec_buffer;
        while (dec_buffer.size() < buffer_size) {
            char receive_byte;
            boost::asio::read(serial, boost::asio::buffer(&receive_byte, 1));
            if (receive_byte) {
                dec_buffer.push_back(static_cast<int>(receive_byte));
            }
        }
        return dec_buffer;
    }

private:
    std::string id;
    boost::asio::serial_port& serial;
    // Other motor attributes like temperature, current, etc., can be added here

    uint16_t modbusCrc(const std::vector<int>& msg) {
        uint16_t crc = 0xFFFF;
        for (auto n : msg) {
            crc ^= n;
            for (int i = 0; i < 8; ++i) {
                if (crc & 1) {
                    crc >>= 1;
                    crc ^= 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

    std::vector<int> calculate_crc(const std::vector<int>& send_buffer) {
        auto crc = modbusCrc(send_buffer);
        // Splitting the CRC into two bytes and appending to the send buffer
        std::vector<int> crc_buffer = send_buffer;
        crc_buffer.push_back(crc & 0xFF);
        crc_buffer.push_back((crc >> 8) & 0xFF);
        return crc_buffer;
    }
};

int main() {
    try {
        boost::asio::io_service io;
        boost::asio::serial_port serial(io, "COM10");
        serial.set_option(boost::asio::serial_port_base::baud_rate(115200));

        RmdMotor motor(1, serial);
        std::vector<int> msg = {0x3E, 0x01, 0x08, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        const int n = 1000;
        auto time_start = std::chrono::high_resolution_clock::now();

        // Example loop, replace with actual logic for progress tracking if needed
        for (int i = 0; i < n; ++i) {
            motor.write(msg);
        }

        auto time_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> time_cost = time_end - time_start;

        std::cout << "Time cost: " << time_cost.count() / 1000 << " s\n";
        std::cout << "Average time cost: " << (time_cost.count() / 1000) / n << " s\n";
        std::cout << "Average frequency: " << n / (time_cost.count() / 1000) << " Hz\n";
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}
