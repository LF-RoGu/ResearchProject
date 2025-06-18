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

class SerialPort
{
    public:
        SerialPort(int deviceId, int baudrate);
        int ReadData(uint8_t *buffer, unsigned int buf_size);
        int get_baudrate();
        mtiDecode_enum find_xsens_device();
        mtiDecode_enum open_xsens_port();
        void xsens_event_handler(XsensEventFlag_t flag, XsensEventData_t* data);
        void set_xsens_data(MTiData xsensData);
        MTiData get_xsens_data(); 
    private:
        int xsensVid;
        int baudrate;
};