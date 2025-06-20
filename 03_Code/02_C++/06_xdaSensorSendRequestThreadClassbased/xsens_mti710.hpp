// xsens_mti710.hpp
#ifndef XSENS_MTI710_HPP
#define XSENS_MTI710_HPP

#include "mti_utility.h"          // brings in MTiData and DEVICE_/OPEN_ codes :contentReference[oaicite:0]{index=0}
#include "libs/xsens_mti.h"
#include "libs/xsens_constants.h"
#include "libs/xsens_utility.h"

#include <atomic>
#include <string>
#include <termios.h>

class XsensMti710 {
public:
    // Find & open return the enums from mti_utility.h
    mtiDecode_enum findXsensDevice();
    mtiDecode_enum openXsensPort();

    void configure();
    void streamDataLoop();

    MTiData getXsensData() const { return xsensData; }
    int     getFd() const       { return fd_; }

    // Allow clean startup
    XsensMti710();
    ~XsensMti710();
    static void xsens_event_handler(XsensEventFlag_t, XsensEventData_t*);

private:
    enum class AckFlag { None, GotoConfig, OutCfg, Flags, GotoMeas };
    std::atomic<AckFlag>        ack_{AckFlag::None};
    
    // singletons for callbacks
    static XsensMti710* instance_;
    static void sendCb(uint8_t*, uint16_t);
    static void cbGotoConfig(xsens_packet_buffer_t*);
    static void cbOutCfg(xsens_packet_buffer_t*);
    static void cbFlags(xsens_packet_buffer_t*);
    static void cbGotoMeas(xsens_packet_buffer_t*);

    // helper
    void waitFor(AckFlag);

    // instance fields
    int                         fd_{-1};
    static inline MTiData                     xsensData{};

    // low-level write
    void sendToDevice(uint8_t* buf, uint16_t len);

    // udev-discovered path & baud
    static constexpr const char* XSENS_VID = "2639"; 
    static inline std::string xsens_device_path = "";            
    static std::string           devNode_;
    static constexpr int BAUDRATE = B115200;
};

#endif // XSENS_MTI710_HPP
