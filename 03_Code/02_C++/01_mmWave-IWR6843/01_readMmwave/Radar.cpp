// Radar.cpp: Definiert den Einstiegspunkt für die Anwendung.
//

#include "Radar.h"

IWR6843 sensor;

const int NUM_THREADS = 3;
pthread_t threads[NUM_THREADS];

const int NUM_FRAMES = 200;
vector<SensorData> totalFrames;

int main() {
    
    cout << "Input filename prefix: " << endl;
    string prefix;
    cin >> prefix;

    //Initializing the sensor
    sensor = IWR6843();
    sensor.init("/dev/ttyUSB0", "/dev/ttyUSB1", "../configs/profile_azim60_elev30_optimized.cfg");
    //sensor.init("/dev/ttyUSB0", "/dev/ttyUSB1", "../configs/profile_2024_11_28T11_50_49_422_azim60_elev30.cfg");
    
    //Creating an array holding the function pointers for the threads
    void* (*thread_functions[NUM_THREADS])(void*) =
    {
        sensor_thread,
        controller_thread,
        actuator_thread
    };
    
    //Creating the threads
    for (int i = 0; i < NUM_THREADS; i++)
    {
        if (pthread_create(&threads[i], nullptr, thread_functions[i], nullptr) != 0)
        {
            cout << "Error creating thread with ID " << i + 1 << endl;
            return -1;
        }
    }

    //Joining the threads
    for (int i = 0; i < NUM_THREADS; i++)
    {
        if (pthread_join(threads[i], nullptr) != 0)
        {
            cout << "Error joining thread with ID " << i + 1 << endl;
            return -1;
        }
    }

    //Returning a 1 after joining all threads (may not be reached but for the sake of completeness)
    cout << "Successfully joined all threads" << endl;

    chrono::time_point<std::chrono::system_clock> timestamp = chrono::system_clock::now();

    string filename = prefix + "_log_" + formatTimestamp(timestamp) + ".csv";

    //Storing the data
    writeToCSV(filename, totalFrames);

    cout << "Log " << filename << " successfully stored" << endl;

    return 1;
}

/// <summary>
/// Function of the sensor thread
/// </summary>
/// <param name="arg"></param>
/// <returns></returns>
void* sensor_thread(void* arg)
{
    //Obtaining the thread's ID
    int thread_id = pthread_self();

    while (true)
    {
        //Polling the sensor and getting the amount of recently received frames
        int numOfNewFrames = sensor.poll();

        //Continuing if no new frames are available
        if (numOfNewFrames < 1)
        {
            continue;
        }

        //Processing if any new frames were received
        //Getting and deleting the new frames from the buffer of decoded frames
        vector<SensorData> newFrames;
        sensor.copyDecodedFramesFromTop(newFrames, numOfNewFrames, true, 100);

        //Inserting it into the vector for logging and exiting if limit was reached
        totalFrames.insert(totalFrames.end(), newFrames.begin(), newFrames.end());
        if (totalFrames.size() >= NUM_FRAMES)
        {
            break;
        }
    }

    //Exiting the thread
    pthread_exit(nullptr);
}

/// <summary>
/// Function of the controller thread
/// </summary>
/// <param name="arg"></param>
/// <returns></returns>
void* controller_thread(void* arg)
{
    //Obtaining the thread's ID
    int thread_id = pthread_self();

    //Simulating work
    cout << "Hello from thread " << thread_id << endl;
    sleep(10);
    
    //Exiting the thread
    pthread_exit(nullptr);
}

/// <summary>
/// Function of the actuator thread
/// </summary>
/// <param name="arg"></param>
/// <returns></returns>
void* actuator_thread(void* arg)
{
    //Obtaining the thread's ID
    int thread_id = pthread_self();
    
    //Simulating work
    cout << "Hello from thread " << thread_id << endl;
    sleep(10);

    
    //Exiting the thread
    pthread_exit(nullptr);
}


std::string formatTimestamp(const std::chrono::time_point<std::chrono::system_clock>& timePoint) {
    auto duration = timePoint.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration) -
        std::chrono::duration_cast<std::chrono::nanoseconds>(seconds);

    std::time_t time = std::chrono::system_clock::to_time_t(timePoint);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time), "%Y-%m-%d");
    oss << "." << std::setw(9) << std::setfill('0') << nanoseconds.count();
    return oss.str();
}

// Convert vector<uint8_t> to a comma-separated string
std::string formatRawData(const std::vector<uint8_t>& data) {
    std::ostringstream oss;
    for (size_t i = 0; i < data.size(); ++i) {
        oss << static_cast<int>(data[i]);
        if (i < data.size() - 1) {
            oss << ",";
        }
    }
    return oss.str();
}



void writeToCSV(const std::string& filename, const std::vector<SensorData>& objects) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    // Write header
    file << "Timestamp,RawData\n";

    // Write each object
    for (const auto& obj : objects) {
        file << formatTimestamp(obj.timestamp) << ",";
        file << "\"" << formatRawData(obj.storedRawData) << "\"\n";  // Enclose RawData in quotes
    }

    file.close();
    std::cout << "Data written to " << filename << std::endl;
}