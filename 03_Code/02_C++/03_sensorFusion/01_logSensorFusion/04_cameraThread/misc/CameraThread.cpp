#include "CameraThread.h"

#include <opencv2/opencv.hpp>
#include <filesystem>  // Requires C++17
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>
#include <iostream>

namespace fs = std::filesystem;

CameraThread::CameraThread(int deviceIndex, int fps)
    : deviceIndex_(deviceIndex),
      frameIntervalMs_(1000 / fps),
      running_(false),
      frameCounter_(0)
{
}

void CameraThread::start()
{
    running_ = true;
    thread_ = std::thread(&CameraThread::run, this);
}

void CameraThread::stop()
{
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void CameraThread::run()
{
    const std::string OUTPUT_FOLDER = "./camera_out/";

    // Create output directory if it doesn't exist
    if (!fs::exists(OUTPUT_FOLDER)) {
        if (!fs::create_directories(OUTPUT_FOLDER)) {
            std::cerr << "[CameraThread] Failed to create output folder: " << OUTPUT_FOLDER << "\n";
            return;
        }
    }

    cap_.open(deviceIndex_);
    if (!cap_.isOpened()) {
        std::cerr << "[CameraThread] Failed to open camera device " << deviceIndex_ << "\n";
        return;
    }

    std::cout << "[CameraThread] Camera started, saving frames to " << OUTPUT_FOLDER << "\n";

    while (running_) {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            std::cerr << "[CameraThread] Failed to capture frame.\n";
            continue;
        }

        // Save the frame
        std::ostringstream filename;
        filename << OUTPUT_FOLDER << "camera_frame_"
                 << std::setw(4) << std::setfill('0') << frameCounter_++
                 << ".jpg";

        if (!cv::imwrite(filename.str(), frame)) {
            std::cerr << "[CameraThread] Failed to write frame to " << filename.str() << "\n";
        } else {
            std::cout << "[CameraThread] Saved: " << filename.str() << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(frameIntervalMs_));
    }

    cap_.release();
    std::cout << "[CameraThread] Camera stopped.\n";
}
