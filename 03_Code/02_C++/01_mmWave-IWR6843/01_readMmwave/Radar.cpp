#include "Radar.h"
#include <unistd.h>    // for sleep()
#include <algorithm>   // for std::min
#include <math.h>      // for std::sqrt

#define PRINT_VALUES

IWR6843    sensor;
const int  NUM_THREADS = 3;
pthread_t  threads[NUM_THREADS];

void* sensor_thread(void* /*arg*/)
{
    while (true)
    {
        int newCount = sensor.poll();
        if (newCount < 0) {
            std::cerr << "[ERROR] sensor.poll() failed\n";
            break;
        }
        if (newCount == 0) {
            continue;
        }

        std::vector<SensorData> frames;
        if (!sensor.copyDecodedFramesFromTop(frames, newCount, true, 100)) {
            std::cerr << "[ERROR] timed out copying frames\n";
            continue;
        }

        for (const SensorData& frame : frames)
        {
            for (const TLVPayloadData& pd : frame.getTLVPayloadData())
            {
                // Safety check for matching counts
                if (pd.SideInfoPoint_str.size() != pd.DetectedPoints_str.size()) {
                    std::cerr << "[ERROR] Mismatch: Detected=" << pd.DetectedPoints_str.size()
                            << " vs SideInfo=" << pd.SideInfoPoint_str.size() << std::endl;
                }

                for (size_t i = 0; i < pd.DetectedPoints_str.size(); ++i)
                {
                    const DetectedPoints& dp = pd.DetectedPoints_str[i];
                    float pt_range = std::sqrt(
                        dp.x_f * dp.x_f + dp.y_f * dp.y_f + dp.z_f * dp.z_f
                    );

                    // Find closest peak in RangeProfilePoint_str
                    float closest_peak_range = -1.0f;
                    uint16_t closest_peak_power = 0;
                    float min_diff = std::numeric_limits<float>::max();

                    for (const RangeProfilePoint& rp : pd.RangeProfilePoint_str) {
                        float diff = std::abs(pt_range - rp.range_f);
                        if (diff < min_diff) {
                            min_diff = diff;
                            closest_peak_range = rp.range_f;
                            closest_peak_power = rp.power_u16;
                        }
                    }

                    bool is_valid = false;
                    if (closest_peak_range >= 0.0f) {
                        // Example condition: must be within 0.1m and power above threshold
                        if (min_diff < 0.1f && closest_peak_power > 3000) {
                            is_valid = true;
                        }
                    }

                    // Show result
                    if (i < pd.SideInfoPoint_str.size()) {
                        const SideInfoPoint& si = pd.SideInfoPoint_str[i];
                        std::cout << "[POINT] X: " << dp.x_f
                                << " Y: " << dp.y_f
                                << " Z: " << dp.z_f
                                << " Doppler: " << dp.doppler_f
                                << " SNR: " << si.snr
                                << " Noise: " << si.noise
                                << " | RANGE: " << pt_range
                                << " | ClosestPeak: " << closest_peak_range
                                << " Pwr: " << closest_peak_power
                                << " | " << (is_valid ? "VALID" : "FILTERED") << "\n";
                    } else {
                        std::cout << "[POINT] X: " << dp.x_f
                                << " Y: " << dp.y_f
                                << " Z: " << dp.z_f
                                << " Doppler: " << dp.doppler_f
                                << " | RANGE: " << pt_range
                                << " | ClosestPeak: " << closest_peak_range
                                << " Pwr: " << closest_peak_power
                                << " | [WARN] Missing SideInfo "
                                << (is_valid ? "VALID" : "FILTERED") << "\n";
                    }
                }
            }
        }


    }

    pthread_exit(nullptr);
}


void* controller_thread(void* /*arg*/)
{
    auto tid = pthread_self();
    cout << "[CTRL] thread " << tid << " running...\n";
    sleep(10);
    pthread_exit(nullptr);
}

void* actuator_thread(void* /*arg*/)
{
    auto tid = pthread_self();
    cout << "[ACT ] thread " << tid << " running...\n";
    sleep(10);
    pthread_exit(nullptr);
}

int main()
{
    // initialize sensor
    cout << "[INFO] Initializing sensor...\n";
    int initResult = sensor.init(
        "/dev/ttyUSB0",
        "/dev/ttyUSB1",
        "./configs/profile_azim60_elev30_optimized.cfg"
    );
    if (initResult != 1) {
        cerr << "[ERROR] sensor.init() returned " << initResult << ":\n";
        switch(initResult) {
            case 0:  cerr << "  • Failed to open config port\n"; break;
            case -1: cerr << "  • Failed to configure config port\n"; break;
            case -2: cerr << "  • Failed to open data port\n"; break;
            case -3: cerr << "  • Failed to configure data port\n"; break;
            case -4: cerr << "  • Failed to send configuration file\n"; break;
            default: cerr << "  • Unknown error\n"; break;
        }
        return initResult;
    }

    // spawn all threads
    void* (*funcs[NUM_THREADS])(void*) = {
        sensor_thread,
        controller_thread,
        actuator_thread
    };
    for (int i = 0; i < NUM_THREADS; ++i) {
        if (pthread_create(&threads[i], nullptr, funcs[i], nullptr) != 0) {
            cerr << "[ERROR] pthread_create() for thread " << i << "\n";
            return -1;
        }
    }

    // join them (sensor_thread never returns, so program runs until killed)
    for (int i = 0; i < NUM_THREADS; ++i) {
        pthread_join(threads[i], nullptr);
    }

    return 0;
}
