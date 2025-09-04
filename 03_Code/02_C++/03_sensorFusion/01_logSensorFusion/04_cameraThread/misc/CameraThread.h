#ifndef CAMERA_THREAD_H
#define CAMERA_THREAD_H

#include <atomic>
#include <thread>
#include <opencv2/opencv.hpp>

class CameraThread {
public:
    CameraThread(int deviceIndex = 0, int fps = 10);
    ~CameraThread();

    void start();
    void stop();
    void captureOnce();  // Optional for timer-based capture

private:
    void run();

    int cameraId_;
    int frameCounter_;
    int frameIntervalMs_;
    int deviceIndex_;
    std::atomic<bool> running_;
    std::thread thread_;
    cv::VideoCapture cap_;
};

#endif // CAMERA_THREAD_H
