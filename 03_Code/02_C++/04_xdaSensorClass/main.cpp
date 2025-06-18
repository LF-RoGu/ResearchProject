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
        std::cerr << "❌ Xsens device not found.\n";
        return DEVICE_FOUND_FAILURE;
    }

    int fd = xsensSensor.open_xsens_port();
    if (fd != OPEN_PORT_SUCCESS) {
        std::cerr << "❌ Failed to open Xsens port.\n";
        return OPEN_PORT_FAILURE;
    }

    // Initialize parser and pass the static event handler
    xsens_interface_t iface = XSENS_INTERFACE_RX(&XsensMti710::xsens_event_handler);
    uint8_t buffer[256];

    std::cout << "✅ Starting Xsens MTi-710 data stream...\n";

    while (true) {
        ssize_t bytesRead = read(fd, buffer, sizeof(buffer));
        if (bytesRead <= 0) {
            std::cerr << "⚠️ Read failed or device disconnected.\n";
            break;
        }

        // Pass buffer to parser
        xsens_mti_parse_buffer(&iface, buffer, bytesRead);

        // Use your class getter
        MTiData data = xsensSensor.get_xsens_data();

        // Print formatted output
        std::cout << "\033[2J\033[H";  // Clear terminal
        std::cout << std::fixed << std::setprecision(3);

        std::cout << "Euler angles: Roll=" << data.euler[0]
                  << ", Pitch=" << data.euler[1]
                  << ", Yaw=" << data.euler[2] << "\n";

        std::cout << "Quaternion: [" << data.quaternion[0] << ", "
                  << data.quaternion[1] << ", "
                  << data.quaternion[2] << ", "
                  << data.quaternion[3] << "]\n";

        std::cout << "Acceleration (m/s²): X=" << data.acceleration[0]
                  << ", Y=" << data.acceleration[1]
                  << ", Z=" << data.acceleration[2] << "\n";

        std::cout << "Free Acceleration (m/s²): X=" << data.free_acceleration[0]
                  << ", Y=" << data.free_acceleration[1]
                  << ", Z=" << data.free_acceleration[2] << "\n";

        std::cout << "Angular Velocity (rad/s): X=" << data.angular_velocity[0]
                  << ", Y=" << data.angular_velocity[1]
                  << ", Z=" << data.angular_velocity[2] << "\n";

        std::cout << "Magnetic Field (µT): X=" << data.magnetic[0]
                  << ", Y=" << data.magnetic[1]
                  << ", Z=" << data.magnetic[2] << "\n";

        std::cout << "Latitude / Longitude: Lat=" << data.latitude
                  << ", Lon=" << data.longitude << "\n";

        std::cout << "Altitude: " << data.altitude << " m\n";

        std::cout << "Velocity (XYZ m/s): X=" << data.velocity[0]
                  << ", Y=" << data.velocity[1]
                  << ", Z=" << data.velocity[2] << "\n";

        std::cout << "Status Byte: 0x" << std::hex << static_cast<int>(data.status_byte) << std::dec << "\n";

        std::cout << "Packet Counter: " << data.packet_counter << "\n";
        std::cout << "Temperature: " << data.temperature << " °C\n";
        std::cout << "Timestamp Fine: " << data.time_fine << "\n";
        std::cout << "Timestamp Coarse: " << data.time_coarse << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    close(fd);
    return 0;
}
