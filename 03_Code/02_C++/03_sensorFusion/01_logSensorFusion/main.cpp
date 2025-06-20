// main.cpp
#include "mti_utility.h"
#include "xsens_mti710.hpp"
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <thread>
#include <chrono>

int main() {
    XsensMti710 sensor;

    if (sensor.findXsensDevice() != DEVICE_FOUND_SUCCESS) {
        std::cerr << "[ERROR] Xsens device not found.\n";
        return DEVICE_FOUND_FAILURE;
    }

    if (sensor.openXsensPort() != OPEN_PORT_SUCCESS) {
        std::cerr << "[ERROR] Failed to open Xsens port.\n";
        return OPEN_PORT_FAILURE;
    }

    sensor.configure();

    // Set up parser for measurement‐only mode
    xsens_interface_t iface = XSENS_INTERFACE_RX(&XsensMti710::xsens_event_handler);

    std::cout << std::fixed << std::setprecision(3);
    uint8_t buf[256];

    while (true) {
        ssize_t n = ::read(sensor.getFd(), buf, sizeof(buf));
        if (n <= 0) {
            std::cerr << "⚠️ Read error or device disconnected.\n";
            break;
        }

        // Feed bytes into the parser, which will update sensor.getXsensData()
        xsens_mti_parse_buffer(&iface, buf, n);

        // Retrieve the latest snapshot
        MTiData d = sensor.getXsensData();

        // Print the fields you care about:
        std::cout << "Acceleration (m/s²): ["
                  << d.acceleration[0] << ", "
                  << d.acceleration[1] << ", "
                  << d.acceleration[2] << "]  "
                  << "Free Accel: ["
                  << d.free_acceleration[0] << ", "
                  << d.free_acceleration[1] << ", "
                  << d.free_acceleration[2] << "]\n";

        std::cout << "Δv (m/s²): ["
                  << d.delta_v[0] << ", "
                  << d.delta_v[1] << ", "
                  << d.delta_v[2] << "]  "
                  << "Δq: ["
                  << d.delta_q[0] << ", "
                  << d.delta_q[1] << ", "
                  << d.delta_q[2] << ", "
                  << d.delta_q[3] << "]\n";

        std::cout << "Rate of Turn (rad/s): ["
                  << d.rate_of_turn[0] << ", "
                  << d.rate_of_turn[1] << ", "
                  << d.rate_of_turn[2] << "]  "
                  << "Magnetic (µT): ["
                  << d.magnetic[0] << ", "
                  << d.magnetic[1] << ", "
                  << d.magnetic[2] << "]\n";

        std::cout << "Temp (°C): " << d.temperature
                  << "  Status Byte: 0x" << std::hex << static_cast<int>(d.status_byte) << std::dec
                  << "  Packet: " << d.packet_counter
                  << "  Time Fine: " << d.time_fine << "\n\n";

        // throttle to your desired print rate
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ::close(sensor.getFd());
    return 0;
}
