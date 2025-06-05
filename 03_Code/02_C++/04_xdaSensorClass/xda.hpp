#ifndef MTI_UTILITY_H
#define MTI_UTILITY_H

#include "mti_utility.h"

extern "C" {
#include "libs/xsens_mti.h"
#include "libs/xsens_constants.h"
#include "libs/xsens_mdata2.h"
#include "libs/xsens_utility.h"
}

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <libudev.h>

class xdaSensorClass {
public:
    xdaSensorClass() = default;
    ~xdaSensorClass() = default;
    bool find_xsens_device();
    int open_xsens_port();
    void xsens_event_handler(XsensEventFlag_t flag, XsensEventData_t* data);
    void get_mti_data(MTiData latestMTiData);
    void set_mti_data(const MTiData& newData);
private:
    std::string xsens_device_path;
    MTiData mtiData_g;
};

#endif // MTI_UTILITY_H
