#ifndef DISTORTIONCORRECTION_H
#define DISTORTIONCORRECTION_H

#include "libcam2opencv.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>
#include "iostream"

using namespace cv;

struct CameraParameters {
    static constexpr double cameraMatrixData[9] = {
       1333.036073634204, 0, 400.4327011660877,
       0, 1340.071081390319, 246.7890719976071,
       0, 0, 1
    };
    static constexpr double distCoeffsData[5] = {
    -0.59546820869468, 2.450259966744147, 0.003972852619769596, -0.001853269499628921, -9.303970854940781
    };
    
};


class DistortionCorrection
{
public:
    DistortionCorrection();
    ~DistortionCorrection();
    void start();
    void stop();
    void addFrame(const Mat& frame);
    void setFrameCallback(std::function<void(const Mat&)> callback);

private:
    void process();
            // The given video and calibration data
    cv::Matx33d cameraMatrix;
    std::vector<double> distCoeffs;
    Mat map1, map2;
    

    std::thread processingThread;
    std::atomic<bool> running;
    std::mutex frameMutex;
    std::condition_variable frameCondition;
    Mat currentFrame;
    bool frameReady;
    std::function<void(const Mat&)> frameCallback;
};

#endif // DISTORTIONCORRECTION_H
