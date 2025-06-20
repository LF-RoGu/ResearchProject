#ifndef RADAR_H
#define RADAR_H

//Global includes
#include <cstdint>
#include <vector>
#include <string>
#include <iostream>
#include <pthread.h>

#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

//Private includes
#include "IWR6843.h"
#include "SensorData.h"

//Namespaces
using namespace std;

//External variables
extern IWR6843 sensor;

extern const int NUM_THREADS;
extern pthread_t threads[];

extern const int NUM_FRAMES;
extern vector<SensorData> totalFrames;

//Function prototypes
void* sensor_thread(void* arg);
void* controller_thread(void* arg);
void* actuator_thread(void* arg);


string formatTimestamp(const std::chrono::time_point<std::chrono::system_clock>& timePoint);
void writeToCSV(const std::string& filename, const std::vector<SensorData>& objects);

#endif // RADAR_H
