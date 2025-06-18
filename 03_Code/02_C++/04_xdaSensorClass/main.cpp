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

    if (xsensSensor.open_xsens_port() != OPEN_PORT_SUCCESS) {
        std::cerr << "❌ Failed to open Xsens port.\n";
        return OPEN_PORT_FAILURE;
    }

    // Get the correct file descriptor
    int fd = xsensSensor.get_fd();

    // Initialize parser and pass the static event handler
    xsens_interface_t iface = XSENS_INTERFACE_RX(&XsensMti710::xsens_event_handler);

    std::cout << "✅ Starting Xsens MTi-710 data stream...\n";

    uint8_t buffer[256];
    ssize_t bytes_read;

    while ((bytes_read = read(fd, buffer, sizeof(buffer))) > 0) {
        xsens_mti_parse_buffer(&iface, buffer, bytes_read);
    }

    if (bytes_read < 0) {
        std::cerr << "⚠️ Read error: " << strerror(errno) << "\n";
    } else {
        std::cerr << "End of stream or device disconnected.\n";
    }

    close(fd);
    return 0;
}
