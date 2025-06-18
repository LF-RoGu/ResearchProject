#pragma once

#include <string>
#include <cstdint>
#include <iostream>
// Drivers used in linux (raspberry pi)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <libudev.h>

#include "mti_utility.h"

extern "C" 
{
    #include "libs/xsens_mti.h"
    #include "libs/xsens_constants.h"
    #include "libs/xsens_mdata2.h"
    #include "libs/xsens_utility.h"
}


class XsensMti710
{
    public:
        XsensMti710();
        XsensMti710(int deviceId, int baudrate);

        mtiDecode_enum find_xsens_device();
        mtiDecode_enum open_xsens_port();

        static void xsens_event_handler(XsensEventFlag_t flag, XsensEventData_t* data);

        static void set_xsens_data(const MTiData& data);
        static MTiData get_xsens_data(); 

    private:
        int xsensVid;
        int baudrate;

        static inline std::string xsens_device_path = "";
        static constexpr int BAUDRATE = B115200;
        static inline MTiData xsensData = {};

        static constexpr const char* XSENS_VID = "2639";
};