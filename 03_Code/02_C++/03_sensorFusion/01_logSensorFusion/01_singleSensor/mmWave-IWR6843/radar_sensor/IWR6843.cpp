#include "IWR6843.h"
#include <chrono>
#include <thread>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <unistd.h>    // for usleep()
#include <sys/ioctl.h> // for FIONREAD

IWR6843::IWR6843() {
    pthread_mutex_init(&decodedFrameBufferMutex, nullptr);
}

IWR6843::~IWR6843() {
    pthread_mutex_destroy(&decodedFrameBufferMutex);
}

/*
 @return  1 on success,
          0 if opening config port fails,
         -1 if configuring config port fails,
         -2 if opening data port fails,
         -3 if configuring data port fails,
         -4 if sending config file fails.
*/
int IWR6843::init(const std::string& configPort,
                  const std::string& dataPort,
                  const std::string& configFilePath)
{
    std::cout << "[INFO] Initializing sensor..." << std::endl;

    std::cout << "[DEBUG] Opening config port: " << configPort << std::endl;
    configPort_fd = open(configPort.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    std::cout << "[DEBUG] configPort_fd = " << configPort_fd << std::endl;
    if (configPort_fd < 0) {
        std::cerr << "[ERROR] Failed to open config port" << std::endl;
        return 0;
    }
    // Flush any pending I/O and configure
    tcflush(configPort_fd, TCIOFLUSH);
    if (configSerialPort(configPort_fd, B115200) < 1) {
        std::cerr << "[ERROR] Failed to configure config port" << std::endl;
        return -1;
    }

    // Give the sensor a moment
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::cout << "[DEBUG] Opening data port: " << dataPort << std::endl;
    dataPort_fd = open(dataPort.c_str(), O_RDONLY | O_NOCTTY | O_SYNC);
    std::cout << "[DEBUG] dataPort_fd = " << dataPort_fd << std::endl;
    if (dataPort_fd < 0) {
        std::cerr << "[ERROR] Failed to open data port" << std::endl;
        return -2;
    }
    tcflush(dataPort_fd, TCIFLUSH);
    if (configSerialPort(dataPort_fd, B921600) < 1) {
        std::cerr << "[ERROR] Failed to configure data port" << std::endl;
        return -3;
    }

    std::cout << "[DEBUG] Sending config file: " << configFilePath << std::endl;
    if (sendConfigFile(configPort_fd, configFilePath) < 1) {
        std::cerr << "[ERROR] Failed to send config file" << std::endl;
        return -4;
    }

    // Give the sensor time to switch into streaming mode
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "[INFO] Sensor initialization complete." << std::endl;
    return 1;
}

int IWR6843::poll()
{
    int bytesAvailable = 0;
    if (ioctl(dataPort_fd, FIONREAD, &bytesAvailable) == -1) {
        return -1;
    }
    if (bytesAvailable < 1) {
        return 0;
    }

    uint8_t buffer[1024];
    int bytesToRead = std::min(bytesAvailable, (int)sizeof(buffer));
    int bytesRead   = read(dataPort_fd, buffer, bytesToRead);
    dataBuffer.insert(dataBuffer.end(), buffer, buffer + bytesRead);

    auto indexesOfMagicWords = findIndexesOfMagicWord();
    if (indexesOfMagicWords.size() < 2) {
        return 0;
    }
    if (indexesOfMagicWords.front() != 0) {
        dataBuffer.erase(dataBuffer.begin(), dataBuffer.begin() + indexesOfMagicWords.front());
    }
    auto sublists = splitIntoSublistsByIndexes(indexesOfMagicWords);

    pthread_mutex_lock(&decodedFrameBufferMutex);
    for (auto& sub : sublists) {
        decodedFrameBuffer.emplace_back(sub);
    }
    pthread_mutex_unlock(&decodedFrameBufferMutex);

    dataBuffer.erase(dataBuffer.begin() + indexesOfMagicWords.front(),
                     dataBuffer.begin() + indexesOfMagicWords.back());

    return sublists.size();
}

bool IWR6843::copyDecodedFramesFromTop(vector<SensorData>& destination,
                                       uint numFrames,
                                       bool delFrames,
                                       long timeout_ms)
{
    timespec timeout;
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec  += timeout_ms / 1000;
    timeout.tv_nsec += (timeout_ms % 1000) * 1000000;
    if (timeout.tv_nsec >= 1000000000) {
        timeout.tv_sec++;
        timeout.tv_nsec -= 1000000000;
    }

    if (pthread_mutex_timedlock(&decodedFrameBufferMutex, &timeout) != 0) {
        return false;
    }
    if (numFrames > decodedFrameBuffer.size()) {
        numFrames = decodedFrameBuffer.size();
    }
    destination.reserve(numFrames);
    copy(decodedFrameBuffer.begin(),
         decodedFrameBuffer.begin() + numFrames,
         back_inserter(destination));
    if (delFrames) {
        decodedFrameBuffer.erase(decodedFrameBuffer.begin(),
                                 decodedFrameBuffer.begin() + numFrames);
    }
    pthread_mutex_unlock(&decodedFrameBufferMutex);
    return true;
}

bool IWR6843::getDecodedFramesSize(int& destination, long timeout_ms)
{
    timespec timeout;
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec  += timeout_ms / 1000;
    timeout.tv_nsec += (timeout_ms % 1000) * 1000000;
    if (timeout.tv_nsec >= 1000000000) {
        timeout.tv_sec++;
        timeout.tv_nsec -= 1000000000;
    }

    if (pthread_mutex_timedlock(&decodedFrameBufferMutex, &timeout) != 0) {
        return false;
    }
    destination = decodedFrameBuffer.size();
    pthread_mutex_unlock(&decodedFrameBufferMutex);
    return true;
}

int IWR6843::configSerialPort(int port_fd, int baudRate)
{
    termios tty{};
    if (tcgetattr(port_fd, &tty) != 0) {
        return -1;
    }
    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(port_fd, TCSANOW, &tty) != 0) {
        return -1;
    }
    return 1;
}

int IWR6843::sendConfigFile(int fd, const std::string& filePath)
{
    std::ifstream cfg(filePath);
    if (!cfg.is_open()) {
        std::cerr << "[ERROR] Could not open config file: " << filePath << "\n";
        return 0;
    }

    std::string line;
    while (std::getline(cfg, line)) {
        if (line.empty() || line[0] == '%') {
            continue;
        }
        // send exactly as in Python, with LF only
        std::string toSend = line + "\n";
        ssize_t n = write(fd, toSend.c_str(), toSend.size());
        if (n < 0) {
            std::cerr << "[ERROR] write(): " << strerror(errno) << "\n";
            return 0;
        }
        // match Python's 10 ms pacing
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "[DEBUG] sendConfigFile: done streaming lines\n";
    return 1;
}

vector<size_t> IWR6843::findIndexesOfMagicWord()
{
    static const vector<uint8_t> pattern = {
        0x02,0x01,0x04,0x03,0x06,0x05,0x08,0x07
    };
    vector<size_t> idx;
    auto it = dataBuffer.begin();
    while ((it = search(it, dataBuffer.end(), pattern.begin(), pattern.end()))
           != dataBuffer.end())
    {
        idx.push_back(distance(dataBuffer.begin(), it));
        ++it;
    }
    return idx;
}

vector<vector<uint8_t>>
IWR6843::splitIntoSublistsByIndexes(const vector<size_t>& indexes)
{
    vector<vector<uint8_t>> sublists;
    for (size_t i = 0; i + 1 < indexes.size(); ++i) {
        auto start = indexes[i];
        auto end   = indexes[i+1];
        sublists.emplace_back(
            dataBuffer.begin() + start,
            dataBuffer.begin() + end - 1
        );
    }
    return sublists;
}
