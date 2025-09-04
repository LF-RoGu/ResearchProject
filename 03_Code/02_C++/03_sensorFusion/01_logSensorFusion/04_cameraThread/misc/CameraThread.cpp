#include "CameraThread.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <filesystem>  // C++17
#include <chrono>
#include <thread>

namespace fs = std::filesystem;

CameraThread::CameraThread(const std::string& devicePath, const std::string& outputDir, int fps)
    : devicePath_(devicePath), outputDir_(outputDir), frameCounter_(0),
      frameIntervalMs_(1000 / fps), running_(false) {}

CameraThread::~CameraThread() {
    stop();
}

void CameraThread::start() {
    running_ = true;
    thread_ = std::thread(&CameraThread::run, this);
}

void CameraThread::stop() {
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
    if (cap_.isOpened()) {
        cap_.release();
    }
}

void CameraThread::captureOnce() {
    if (!cap_.isOpened()) {
        cap_.open(devicePath_);
        if (!cap_.isOpened()) {
            std::cerr << "[CameraThread] Failed to open camera for one-shot capture.\n";
            return;
        }
    }

    cv::Mat frame;
    if (cap_.read(frame)) {
        std::ostringstream filename;
        filename << outputDir_ << "/frame_" << std::setw(4) << std::setfill('0') << frameCounter_++ << ".jpg";
        cv::imwrite(filename.str(), frame);
    } else {
        std::cerr << "[CameraThread] One-shot capture failed.\n";
    }
}

void CameraThread::run() {
    // Ensure output directory exists
    if (!fs::exists(outputDir_)) {
        if (!fs::create_directories(outputDir_)) {
            std::cerr << "[CameraThread] Failed to create directory: " << outputDir_ << "\n";
            return;
        }
    }

    cap_.open(devicePath_);
    if (!cap_.isOpened()) {
        std::cerr << "[CameraThread] Failed to open camera: " << devicePath_ << "\n";
        return;
    }

    std::cout << "[CameraThread] Started capturing from: " << devicePath_ << "\n";

    while (running_) {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            std::cerr << "[CameraThread] Failed to capture frame.\n";
            continue;
        }

        // Save the frame with padded counter
        std::ostringstream filename;
        filename << outputDir_ << "/frame_" << std::setw(4) << std::setfill('0') << frameCounter_++ << ".jpg";
        if (!cv::imwrite(filename.str(), frame)) {
            std::cerr << "[CameraThread] Failed to write image: " << filename.str() << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(frameIntervalMs_));
    }

    cap_.release();
    std::cout << "[CameraThread] Camera stopped.\n";
}
