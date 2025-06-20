#include "mti_utility.h"
#include "xsens_mti710.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <unistd.h>

int main() {
    XsensMti710 xsensSensor;

    if (xsensSensor.find_xsens_device() != DEVICE_FOUND_SUCCESS) {
        std::cerr << "[ERROR] Xsens device not found.\n";
        return DEVICE_FOUND_FAILURE;
    }

    if (xsensSensor.open_xsens_port() != OPEN_PORT_SUCCESS) {
        std::cerr << "[ERROR] Failed to open Xsens port.\n";
        return OPEN_PORT_FAILURE;
    }

    // Initialize parser and pass the static event handler
    xsens_interface_t iface = XSENS_INTERFACE_RX(&XsensMti710::xsens_event_handler);

    std::cout << "Starting Xsens MTi-710 data stream...\n";

    uint8_t buffer[256];
    ssize_t bytes_read;

    while (true) {
        int fd = xsensSensor.get_fd();  // 👈 get the file descriptor from the class
        bytes_read = read(fd, buffer, sizeof(buffer));

        if (bytes_read > 0) {
            xsens_mti_parse_buffer(&iface, buffer, bytes_read);

            // Use your class getter to access the most recent data
            MTiData data = xsensSensor.get_xsens_data();

            std::cout << "\033[2J\033[H";  // Clear terminal screen
/*             std::cout << std::fixed << std::setprecision(3);

            std::cout << "====== Sensor Snapshot ======\n";
            std::cout << "Euler angles [rad]:\n"
                    << "  Roll  = " << data.euler[0]
                    << ", Pitch = " << data.euler[1]
                    << ", Yaw   = " << data.euler[2] << "\n";

            std::cout << "Quaternion [w, x, y, z]:\n"
                    << "  [" << data.quaternion[0] << ", "
                            << data.quaternion[1] << ", "
                            << data.quaternion[2] << ", "
                            << data.quaternion[3] << "]\n";

            std::cout << "Acceleration (m/s²):\n"
                    << "  X = " << data.acceleration[0]
                    << ", Y = " << data.acceleration[1]
                    << ", Z = " << data.acceleration[2] << "\n";

            std::cout << "Free Acceleration (m/s²):\n"
                    << "  X = " << data.free_acceleration[0]
                    << ", Y = " << data.free_acceleration[1]
                    << ", Z = " << data.free_acceleration[2] << "\n";

            std::cout << "Status Byte: 0x" << std::hex << static_cast<int>(data.status_byte) << std::dec << "\n";
            std::cout << "==============================\n"; */
    
        } else {
            std::cerr << "⚠️ Read error or device disconnected.\n";
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    close(xsensSensor.get_fd());
    return 0;
}
