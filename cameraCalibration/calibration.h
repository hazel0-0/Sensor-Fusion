#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <atomic>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "libcam2opencv.h"


class Calibration {
public:
    Calibration();
    ~Calibration();

    void start();
    void stop();
    void addFrame(const cv::Mat& frame);
    void stopCollection();

private:
    void processImages();
    void performCalibration();

    std::vector<cv::Mat> images;
    cv::Size boardSize;
    float squareSize;

    std::thread calibrationThread;
    std::atomic<bool> running;
    std::atomic<bool> frameReady;
    std::mutex frameMutex;
    std::condition_variable frameCondition;
    cv::Mat currentFrame;
    std::vector<std::vector<cv::Point2f>> imagePoints; 
    std::mutex mtx;
};

#endif // CALIBRATION_H
