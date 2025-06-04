#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <libudev.h>
#include <cstring>
#include <vector>
#include <cstdint>
#include <iomanip>
#include <cmath>
#include "mti_decode.h"

#define XSENS_VID "2639"
#define BAUDRATE B115200

std::string xsens_device_path;

mtiDecode_enum find_xsens_device() {
    struct udev* udev = udev_new();
    if (udev == nullptr) {
        std::cerr << "❌ Failed to create udev context.\n";
        return DEVICE_FOUND_FAILURE;
    }

    struct udev_enumerate* enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_scan_devices(enumerate);

    struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry* entry;

    udev_list_entry_foreach(entry, devices) {
        const char* path = udev_list_entry_get_name(entry);
        struct udev_device* dev = udev_device_new_from_syspath(udev, path);
        struct udev_device* parent = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");

        if (parent != nullptr) {
            const char* vid = udev_device_get_sysattr_value(parent, "idVendor");
            const char* dev_node = udev_device_get_devnode(dev);

            if (vid != nullptr && dev_node != nullptr) {
                std::string vid_str(vid);
                if (vid_str == XSENS_VID) {
                    xsens_device_path = std::string(dev_node);
                    std::cout << "✅ Xsens device found: " << xsens_device_path << "\n";
                    udev_device_unref(dev);
                    udev_unref(udev);
                    return DEVICE_FOUND_SUCCESS;
                }
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

void read_xbus_message_start(int fd) {
    std::cout << "📥 Reading Xbus message header...\n";

    uint8_t fields[4];
    if (read(fd, fields, 4) != 4 || fields[0] != 0xFA) {
        std::cerr << "❌ Invalid Xbus message preamble.\n";
        return;
    }

    std::cout << std::hex << std::setfill('0');
    std::cout << "📄 Header → Preamble: 0x" << std::setw(2) << (int)fields[0]
              << ", BID: 0x" << std::setw(2) << (int)fields[1]
              << ", MID: 0x" << std::setw(2) << (int)fields[2]
              << ", LEN: 0x" << std::setw(2) << (int)fields[3] << "\n";
}

int main() {
    if (find_xsens_device() != DEVICE_FOUND_SUCCESS) return DEVICE_FOUND_FAILURE;

    int fd = open_xsens_port();
    if (fd < 0) return OPEN_PORT_FAILURE;

    read_xbus_message_start(fd);
    close(fd);
    return OPEN_PORT_SUCCESS;
}
