#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <libudev.h>
#include <cstring>
#include <vector>
#include <cstdint>
#include <iomanip>
#include "mti_decode.h"

#define XSENS_VID "2639"
#define BAUDRATE B115200

std::string xsens_device_path;

mtiDecode_enum find_xsens_device() {
    struct udev* udev = udev_new();
    if (!udev) return DEVICE_FOUND_FAILURE;

    struct udev_enumerate* enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_scan_devices(enumerate);

    struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry* entry;

    udev_list_entry_foreach(entry, devices) {
        const char* path = udev_list_entry_get_name(entry);
        struct udev_device* dev = udev_device_new_from_syspath(udev, path);
        struct udev_device* parent = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");

        if (parent) {
            const char* vid = udev_device_get_sysattr_value(parent, "idVendor");
            const char* dev_node = udev_device_get_devnode(dev);
            if (vid && dev_node && std::string(vid) == XSENS_VID) {
                xsens_device_path = std::string(dev_node);
                std::cout << "✅ Xsens device found: " << xsens_device_path << "\n";
                udev_device_unref(dev);
                udev_unref(udev);
                return DEVICE_FOUND_SUCCESS;
            }
        }
        udev_device_unref(dev);
    }

    udev_unref(udev);
    std::cerr << "❌ No Xsens device found.\n";
    return DEVICE_FOUND_FAILURE;
}

int open_xsens_port() {
    int fd = open(xsens_device_path.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::perror("❌ Failed to open port");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::perror("❌ Failed to get port attributes");
        close(fd);
        return -1;
    }

    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);
    tty.c_cflag = CS8 | CLOCAL | CREAD;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 1;

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::perror("❌ Failed to set port attributes");
        close(fd);
        return -1;
    }

    std::cout << "🔌 Serial port opened successfully.\n";
    return fd;
}

bool read_xbus_message_start(int fd, XbusMessage& msg) {
    uint8_t byte = 0;

    // Poll until preamble (0xFA)
    while (true) {
        if (read(fd, &byte, 1) == 1 && byte == 0xFA) {
            msg.preamble = byte;
            std::cout << "Preamble found: 0x" << std::hex << (int)msg.preamble << "\n";
            break;
        }
    }

    // Read BID, MID, LEN
    uint8_t header[3];
    if (read(fd, header, 3) != 3) return false;
    msg.bid = header[0];
    msg.mid = header[1];
    msg.len = header[2];

    std::cout << "BID: 0x" << std::hex << (int)msg.bid
              << ", MID: 0x" << (int)msg.mid
              << ", LEN: 0x" << (int)msg.len << "\n";

    // Determine payload length
    size_t payload_len = 0;
    if (msg.len == 0xFF) {
        uint8_t ext_len_bytes[2];
        if (read(fd, ext_len_bytes, 2) != 2) return false;
        msg.ext_len = (ext_len_bytes[0] << 8) | ext_len_bytes[1];
        payload_len = msg.ext_len;
        std::cout << "Extended length detected: " << msg.ext_len << " bytes\n";
    } else {
        msg.ext_len = 0;
        payload_len = msg.len;
    }

    // Read payload
    msg.data.resize(payload_len);
    if (read(fd, msg.data.data(), payload_len) != (ssize_t)payload_len) return false;

    std::cout << "Payload (" << payload_len << " bytes): ";
    for (size_t i = 0; i < payload_len; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)msg.data[i] << " ";
    }
    std::cout << "\n";

    // Read checksum
    if (read(fd, &msg.checksum, 1) != 1) return false;
    std::cout << "Read checksum: 0x" << std::hex << (int)msg.checksum << "\n";

    // Correct Checksum Validation (sum all fields including checksum, must end in 0)
    uint16_t sum = 0;
    sum += msg.bid;
    sum += msg.mid;
    sum += msg.len;

    if (msg.len == 0xFF) {
        sum += (msg.ext_len >> 8) & 0xFF;
        sum += msg.ext_len & 0xFF;
    }

    for (auto b : msg.data) sum += b;
    sum += msg.checksum;

    if ((sum & 0xFF) != 0) {
        std::cerr << "Checksum invalid. Sum LSB: 0x" << std::hex << (sum & 0xFF) << "\n";
        return false;
    }

    std::cout << "✅ Checksum valid.\n";
    return true;
}


int main() {
    if (find_xsens_device() != DEVICE_FOUND_SUCCESS)
        return DEVICE_FOUND_FAILURE;

    int fd = open_xsens_port();
    if (fd < 0)
        return OPEN_PORT_FAILURE;

    XbusMessage msg;
    if (read_xbus_message_start(fd, msg)) {
        // TODO: process msg
    }

    close(fd);
    return OPEN_PORT_SUCCESS;
}
