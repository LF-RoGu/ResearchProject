#ifndef RADAR_H
#define RADAR_H

#include <cstdint>
#include <vector>
#include <string>
#include <iostream>
#include <pthread.h>
#include <chrono>
#include <iomanip>
#include <sstream>

#include "radar_sensor/IWR6843.h"
#include "radar_sensor/SensorData.h"

using namespace std;

// single global sensor instance
extern IWR6843 sensor;

// thread count & handles
extern const int    NUM_THREADS;
extern pthread_t    threads[];

// thread functions
void* sensor_thread(void* arg);
void* controller_thread(void* arg);
void* actuator_thread(void* arg);

// timestamp formatting
string formatTimestamp(const chrono::time_point<chrono::system_clock>& tp);

#endif // RADAR_H
