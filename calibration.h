#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <atomic>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "libcam2opencv.h"
#include "iostream"

class Calibration {
public:
    Calibration();
    ~Calibration();

    void start();
    void stop();
    void addFrame(const cv::Mat& frame);
    void stopCollection();
    void setFrameCallback(std::function<void(const cv::Mat&)> callback);
    void performCalibration();

private:
    void processImages();

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
    std::function<void(const cv::Mat&)> frameCallback;
    
};

#endif // CALIBRATION_H
