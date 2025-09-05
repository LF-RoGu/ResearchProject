#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

using namespace std;

queue<cv::Mat> frameQueue;
mutex queueMutex;
condition_variable frameAvailable;
atomic<bool> stopSignal(false);

const string outputPath = "_outFiles/video/output_threaded_keypress.avi";

// Set stdin to non-canonical mode (no Enter required)
void setNonBlockingInput(bool enable) {
    static struct termios oldt, newt;
    if (enable) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
}

void threadKeyboard() {
    cout << "[KEYBOARD] Press 'q' to stop recording..." << endl;
    setNonBlockingInput(true);

    while (!stopSignal.load()) {
        fd_set set;
        struct timeval timeout;
        FD_ZERO(&set);
        FD_SET(STDIN_FILENO, &set);

        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; // 100 ms

        int res = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
        if (res > 0) {
            char ch;
            read(STDIN_FILENO, &ch, 1);
            if (ch == 'q') {
                cout << "[KEYBOARD] 'q' pressed. Stopping recording." << endl;
                stopSignal.store(true);
                frameAvailable.notify_all(); // Wake up saving thread
                break;
            }
        }
    }

    setNonBlockingInput(false);
    cout << "[KEYBOARD] Thread exiting." << endl;
}

void threadCapture(cv::VideoCapture& cap) {
    cout << "[CAPTURE] Thread started." << endl;
    while (!stopSignal.load()) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            cerr << "[CAPTURE] Empty frame detected." << endl;
            continue;
        }

        {
            lock_guard<mutex> lock(queueMutex);
            frameQueue.push(frame.clone());
        }

        frameAvailable.notify_one();
        this_thread::sleep_for(chrono::milliseconds(50));  // ~20 FPS
    }
    cout << "[CAPTURE] Thread exiting." << endl;
}

void threadSave(int width, int height) {
    cout << "[SAVE] Thread started." << endl;

    cv::VideoWriter writer(outputPath, cv::VideoWriter::fourcc('M','J','P','G'), 20, cv::Size(width, height));
    if (!writer.isOpened()) {
        cerr << "[SAVE] Failed to open output file." << endl;
        return;
    }

    while (!stopSignal.load() || !frameQueue.empty()) {
        unique_lock<mutex> lock(queueMutex);
        frameAvailable.wait(lock, []{ return !frameQueue.empty() || stopSignal.load(); });

        while (!frameQueue.empty()) {
            cv::Mat frame = frameQueue.front();
            frameQueue.pop();
            lock.unlock();

            writer.write(frame);

            lock.lock();
        }
    }

    writer.release();
    cout << "[SAVE] Thread exiting. File saved to: " << outputPath << endl;
}

int main() {
    cout << "[MAIN] Starting threaded webcam recording with key interrupt..." << endl;

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "[MAIN] Could not open webcam." << endl;
        return 1;
    }

    int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    cout << "[MAIN] Resolution: " << width << "x" << height << endl;

    thread t1(threadCapture, ref(cap));
    thread t2(threadSave, width, height);
    thread t3(threadKeyboard);  // Listen for 'q'

    t1.join();
    t2.join();
    t3.join();

    cap.release();

    cout << "[MAIN] Recording finished." << endl;
    return 0;
}
