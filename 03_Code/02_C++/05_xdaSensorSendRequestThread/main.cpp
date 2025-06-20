#include "mti_utility.h"
#include "xsens_mti710.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <unistd.h> // for read/write
#include <atomic>

// --- 1) ACK‐flag bookkeeping --------------------------------
enum class AckFlag {
    None,
    GotoconfigAck,
    OutputConfigAck,
    OptionFlagsAck,
    GotomeasurementAck
};
static std::atomic<AckFlag> ack_flag{AckFlag::None};

// 2) Your ACK callbacks:
extern "C" {
void handle_gotoconfig_ack(xsens_packet_buffer_t*)      { ack_flag = AckFlag::GotoconfigAck; }
void handle_outputconfig_ack(xsens_packet_buffer_t*)   { ack_flag = AckFlag::OutputConfigAck; }
void handle_optionflags_ack(xsens_packet_buffer_t*)    { ack_flag = AckFlag::OptionFlagsAck; }
void handle_gotomeasurement_ack(xsens_packet_buffer_t*){ ack_flag = AckFlag::GotomeasurementAck; }
}

// 3) Global pointer so sendToDevice can see the fd:
static XsensMti710* gXsensPtr = nullptr;

// 4) Wrapper that writes MTi‐packets out your serial FD
void sendToDevice(uint8_t* data, uint16_t len)
{
    if (gXsensPtr) {
        write(gXsensPtr->get_fd(), data, len);
    }
}

// 5) Configuration routine
void configureSensor(XsensMti710& xsensSensor)
{
    // stash pointer so sendToDevice() can see it
    gXsensPtr = &xsensSensor;

    int fd = xsensSensor.get_fd();

    // Build an interface that both RXes and TXes
    xsens_interface_t iface =
      XSENS_INTERFACE_RX_TX(
        &XsensMti710::xsens_event_handler,
        &sendToDevice
      );

    // Override the internal handlers so we catch the ACK packets
    xsens_mti_override_id_handler(MT_ACK_GOTOCONFIG,          handle_gotoconfig_ack);
    xsens_mti_override_id_handler(MT_ACK_OUTPUTCONFIGURATION,handle_outputconfig_ack);
    xsens_mti_override_id_handler(MT_ACK_OPTIONFLAGS,        handle_optionflags_ack);
    xsens_mti_override_id_handler(MT_ACK_GOTOMEASUREMENT,    handle_gotomeasurement_ack);

    uint8_t buffer[256];
    ssize_t n;

    // A) Go into config mode
    ack_flag = AckFlag::None;
    xsens_mti_request(&iface, MT_GOTOCONFIG);
    while (ack_flag != AckFlag::GotoconfigAck) {
        n = read(fd, buffer, sizeof(buffer));
        if (n > 0) xsens_mti_parse_buffer(&iface, buffer, n);
    }

    // B) Send YOUR output configuration (matches your GUI screenshot)
    const uint16_t RATE = 10;  // 10 Hz
    XsensFrequencyConfig_t cfg[] = {
      { .id = XDI_PACKET_COUNTER,      .frequency = 0xFFFF },
      { .id = XDI_SAMPLE_TIME_FINE,    .frequency = 0xFFFF },
      { .id = XDI_DELTA_Q,             .frequency = RATE },
      { .id = XDI_RATE_OF_TURN,        .frequency = RATE },
      { .id = XDI_DELTA_V,             .frequency = RATE },
      { .id = XDI_ACCELERATION,        .frequency = RATE },
      { .id = XDI_FREE_ACCELERATION,   .frequency = RATE },
      { .id = XDI_MAGNETIC_FIELD,      .frequency = RATE },
      { .id = XDI_TEMPERATURE,         .frequency = RATE },
      { .id = XDI_STATUS_BYTE,         .frequency = RATE },
      { .id = XSENS_IDENTIFIER_FORMAT(
                    XDI_ALTITUDE_ELLIPSOID,
                    XSENS_FLOAT_FIXED1632,
                    XSENS_COORD_ENU),
        .frequency = RATE
      },
    };
    ack_flag = AckFlag::None;
    xsens_mti_set_configuration(&iface, cfg, sizeof(cfg)/sizeof(cfg[0]));
    while (ack_flag != AckFlag::OutputConfigAck) {
        n = read(fd, buffer, sizeof(buffer));
        if (n > 0) xsens_mti_parse_buffer(&iface, buffer, n);
    }

    // C) Set option flags: disable autostore, enable AHS
    uint32_t setFlags = 0, clearFlags = 0;
    XSENS_OPTION_FLAG_SET(setFlags, XSENS_OPTFLAG_DISABLE_AUTOSTORE);
    XSENS_OPTION_FLAG_SET(setFlags, XSENS_OPTFLAG_ENABLE_AHS);
    ack_flag = AckFlag::None;
    xsens_mti_set_option_flags(&iface, setFlags, clearFlags);
    while (ack_flag != AckFlag::OptionFlagsAck) {
        n = read(fd, buffer, sizeof(buffer));
        if (n > 0) xsens_mti_parse_buffer(&iface, buffer, n);
    }

    // D) Finally go back to measurement mode
    ack_flag = AckFlag::None;
    xsens_mti_request(&iface, MT_GOTOMEASUREMENT);
    while (ack_flag != AckFlag::GotomeasurementAck) {
        n = read(fd, buffer, sizeof(buffer));
        if (n > 0) xsens_mti_parse_buffer(&iface, buffer, n);
    }
}

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

    // Push your custom configuration:
    configureSensor(xsensSensor);

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
