#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <libudev.h>
#include <iomanip>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

extern "C" {
#include "libs/xsens_mti.h"
#include "libs/xsens_constants.h"
#include "libs/xsens_mdata2.h"
#include "libs/xsens_utility.h"
}
#include "mti_utility.h"

#define XSENS_VID "2639"
#define BAUDRATE B115200

std::string xsens_device_path;
MTiData mtiData_g;

std::mutex mti_mutex;
std::condition_variable mti_cv;
std::atomic<bool> data_ready(false);
std::atomic<bool> stop_threads(false);

// Device discovery
bool find_xsens_device() {
    struct udev* udev = udev_new();
    if (!udev) return false;

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
                return true;
            }
        }
        udev_device_unref(dev);
    }

    udev_unref(udev);
    std::cerr << "No Xsens device found.\n";
    return false;
}

// Open and configure port
int open_xsens_port() {
    int fd = open(xsens_device_path.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::perror("Failed to open port");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::perror("Failed to get port attributes");
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
        std::perror("Failed to set port attributes");
        close(fd);
        return -1;
    }

    std::cout << "Serial port opened successfully.\n";
    return fd;
}

// Event callback
void xsens_event_handler(XsensEventFlag_t flag, XsensEventData_t* data) {
    std::lock_guard<std::mutex> lock(mti_mutex);

    if (flag & XSENS_EVT_EULER) {
        mtiData_g.euler[0] = data->data.f4x3[0];
        mtiData_g.euler[1] = data->data.f4x3[1];
        mtiData_g.euler[2] = data->data.f4x3[2];
    }

    if (flag & XSENS_EVT_QUATERNION) {
        mtiData_g.quaternion[0] = data->data.f4x4[0];
        mtiData_g.quaternion[1] = data->data.f4x4[1];
        mtiData_g.quaternion[2] = data->data.f4x4[2];
        mtiData_g.quaternion[3] = data->data.f4x4[3];
    }

    if (flag & XSENS_EVT_ACCELERATION) {
        mtiData_g.acceleration[0] = data->data.f4x3[0];
        mtiData_g.acceleration[1] = data->data.f4x3[1];
        mtiData_g.acceleration[2] = data->data.f4x3[2];
    }

    if (flag & XSENS_EVT_FREE_ACCELERATION) {
        mtiData_g.free_acceleration[0] = data->data.f4x3[0];
        mtiData_g.free_acceleration[1] = data->data.f4x3[1];
        mtiData_g.free_acceleration[2] = data->data.f4x3[2];
    }

    if (flag & XSENS_EVT_RATE_OF_TURN) {
        mtiData_g.angular_velocity[0] = data->data.f4x3[0];
        mtiData_g.angular_velocity[1] = data->data.f4x3[1];
        mtiData_g.angular_velocity[2] = data->data.f4x3[2];
    }

    if (flag & XSENS_EVT_MAGNETIC) {
        mtiData_g.magnetic[0] = data->data.f4x3[0];
        mtiData_g.magnetic[1] = data->data.f4x3[1];
        mtiData_g.magnetic[2] = data->data.f4x3[2];
    }

    if (flag & XSENS_EVT_LAT_LON) {
        mtiData_g.latitude = data->data.f8x2[0];
        mtiData_g.longitude = data->data.f8x2[1];
    }

    if (flag & XSENS_EVT_ALTITUDE_ELLIPSOID) {
        mtiData_g.altitude = data->data.f8;
    }

    if (flag & XSENS_EVT_VELOCITY_XYZ) {
        mtiData_g.velocity[0] = data->data.f4x3[0];
        mtiData_g.velocity[1] = data->data.f4x3[1];
        mtiData_g.velocity[2] = data->data.f4x3[2];
    }

    if (flag & XSENS_EVT_STATUS_BYTE) {
        mtiData_g.status_byte = static_cast<int>(data->data.u1);
    }

    if (flag & XSENS_EVT_PACKET_COUNT) {
        mtiData_g.packet_counter = data->data.u2;
    }

    if (flag & XSENS_EVT_TEMPERATURE) {
        mtiData_g.temperature = data->data.f4;
    }

    if (flag & XSENS_EVT_TIME_FINE) {
        mtiData_g.time_fine = data->data.u4;
    }

    if (flag & XSENS_EVT_TIME_COARSE) {
        mtiData_g.time_coarse = data->data.u4;
    }

    data_ready = true;
    mti_cv.notify_one();
}

// Read thread
void read_thread_func(int fd, xsens_interface_t* iface) {
    uint8_t buffer[256];
    ssize_t bytes_read;

    while (!stop_threads.load()) {
        bytes_read = read(fd, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            xsens_mti_parse_buffer(iface, buffer, bytes_read);
        } else {
            stop_threads = true;
            break;
        }
    }
}

// Print thread
void print_thread_func() {
    while (!stop_threads.load()) {
        std::unique_lock<std::mutex> lock(mti_mutex);
        mti_cv.wait(lock, [] { return data_ready.load() || stop_threads.load(); });

        if (stop_threads.load()) break;

        std::cout << "\033[2J\033[H";
        std::cout << std::fixed << std::setprecision(6);

        std::cout << "Euler angles (rad): "
                  << "Roll=" << mtiData_g.euler[0]
                  << ", Pitch=" << mtiData_g.euler[1]
                  << ", Yaw=" << mtiData_g.euler[2] << "\n";

        std::cout << "Quaternion: ["
                  << mtiData_g.quaternion[0] << ", "
                  << mtiData_g.quaternion[1] << ", "
                  << mtiData_g.quaternion[2] << ", "
                  << mtiData_g.quaternion[3] << "]\n";

        std::cout << "Acceleration (m/s²): "
                  << "X=" << mtiData_g.acceleration[0]
                  << ", Y=" << mtiData_g.acceleration[1]
                  << ", Z=" << mtiData_g.acceleration[2] << "\n";

        std::cout << "Free Acceleration (m/s²): "
                  << "X=" << mtiData_g.free_acceleration[0]
                  << ", Y=" << mtiData_g.free_acceleration[1]
                  << ", Z=" << mtiData_g.free_acceleration[2] << "\n";

        std::cout << "Angular Velocity (rad/s): "
                  << "X=" << mtiData_g.angular_velocity[0]
                  << ", Y=" << mtiData_g.angular_velocity[1]
                  << ", Z=" << mtiData_g.angular_velocity[2] << "\n";

        std::cout << "Magnetic Field (µT): "
                  << "X=" << mtiData_g.magnetic[0]
                  << ", Y=" << mtiData_g.magnetic[1]
                  << ", Z=" << mtiData_g.magnetic[2] << "\n";

        std::cout << "Latitude / Longitude: "
                  << "Lat=" << mtiData_g.latitude
                  << ", Lon=" << mtiData_g.longitude << "\n";

        std::cout << "Altitude (ellipsoidal): " << mtiData_g.altitude << " m\n";

        std::cout << "Velocity (XYZ m/s): "
                  << "X=" << mtiData_g.velocity[0]
                  << ", Y=" << mtiData_g.velocity[1]
                  << ", Z=" << mtiData_g.velocity[2] << "\n";

        std::cout << "Status Byte: 0x" << std::hex << static_cast<int>(mtiData_g.status_byte) << std::dec << "\n";

        std::cout << "Packet Counter: " << mtiData_g.packet_counter << "\n";

        std::cout << "Temperature: " << mtiData_g.temperature << " °C\n";

        std::cout << "Timestamp (Fine): " << mtiData_g.time_fine << " ticks\n";
        std::cout << "Timestamp (Coarse): " << mtiData_g.time_coarse << " ms\n";

        data_ready = false;
        usleep(10);
    }
}

int main() {
    if (!find_xsens_device()) return 1;

    int fd = open_xsens_port();
    if (fd < 0) return 1;

    std::cout << "Reading MTi-710 stream...\n";
    xsens_interface_t iface = XSENS_INTERFACE_RX(&xsens_event_handler);

    std::thread reader(read_thread_func, fd, &iface);
    std::thread printer(print_thread_func);

    reader.join();
    stop_threads = true;
    mti_cv.notify_one();
    printer.join();

    close(fd);
    return 0;
}
