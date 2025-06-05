#include "xda.hpp"
#include "mti_utility.h"
#include <thread>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <unistd.h>

int main() {
    xdaSensorClass xsensSensor;

    if (!xsensSensor.find_xsens_device()) return 1;

    int fd = xsensSensor.open_xsens_port();
    if (fd < 0) return 1;

    MTiData currentData;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Reading MTi-710 data every 0.1 seconds \n";

    while (true) {
        xsensSensor.get_mti_data(currentData);

        std::cout << "\033[2J\033[H";  // Clear terminal

        std::cout << "Euler angles: Roll=" << currentData.euler[0]
                  << ", Pitch=" << currentData.euler[1]
                  << ", Yaw=" << currentData.euler[2] << "\n";

        std::cout << "Quaternion: ["
                  << currentData.quaternion[0] << ", "
                  << currentData.quaternion[1] << ", "
                  << currentData.quaternion[2] << ", "
                  << currentData.quaternion[3] << "]\n";

        std::cout << "Acceleration (m/s²): X=" << currentData.acceleration[0]
                  << ", Y=" << currentData.acceleration[1]
                  << ", Z=" << currentData.acceleration[2] << "\n";

        std::cout << "Free Acceleration (m/s²): X=" << currentData.free_acceleration[0]
                  << ", Y=" << currentData.free_acceleration[1]
                  << ", Z=" << currentData.free_acceleration[2] << "\n";

        std::cout << "Angular Velocity (rad/s): X=" << currentData.angular_velocity[0]
                  << ", Y=" << currentData.angular_velocity[1]
                  << ", Z=" << currentData.angular_velocity[2] << "\n";

        std::cout << "Magnetic Field (µT): X=" << currentData.magnetic[0]
                  << ", Y=" << currentData.magnetic[1]
                  << ", Z=" << currentData.magnetic[2] << "\n";

        std::cout << "Latitude / Longitude: Lat=" << currentData.latitude
                  << ", Lon=" << currentData.longitude << "\n";

        std::cout << "Altitude: " << currentData.altitude << " m\n";

        std::cout << "Velocity (XYZ m/s): X=" << currentData.velocity[0]
                  << ", Y=" << currentData.velocity[1]
                  << ", Z=" << currentData.velocity[2] << "\n";

        std::cout << "Status Byte: 0x" << std::hex << static_cast<int>(currentData.status_byte) << std::dec << "\n";

        std::cout << "Packet Counter: " << currentData.packet_counter << "\n";
        std::cout << "Temperature: " << currentData.temperature << " °C\n";
        std::cout << "Timestamp Fine: " << currentData.time_fine << "\n";
        std::cout << "Timestamp Coarse: " << currentData.time_coarse << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    close(fd);
    return 0;
}
