#ifndef CAMERA_THREAD_H
#define CAMERA_THREAD_H

#include <atomic>
#include <thread>
#include <string>
#include <opencv2/opencv.hpp>

class CameraThread {
public:
    CameraThread(const std::string& devicePath, const std::string& outputDir = "camera_output", int fps = 10);
    ~CameraThread();

    void start();
    void stop();
    void captureOnce();  // Optional

private:
    void run();

    std::string devicePath_;
    std::string outputDir_;
    int frameCounter_;
    int frameIntervalMs_;
    std::atomic<bool> running_;
    std::thread thread_;
    cv::VideoCapture cap_;
};

#endif // CAMERA_THREAD_H
