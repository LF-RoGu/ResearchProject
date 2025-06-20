#include "mti_utility.h"
#include "xsens_mti710.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <unistd.h> // for read/write
#include <atomic>

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

    sensor.configure();         // goToConfig → setOutputConfig → setOptionFlags → goToMeasurement
    sensor.streamDataLoop();    // enter continuous read/parse

    return 0;
}