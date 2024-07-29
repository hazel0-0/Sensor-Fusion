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
       1292.243579532522, 0, 444.8808116818876,
       0, 1296.654772464041, 201.9437724229773,
       0, 0, 1
    };

    static constexpr double distCoeffsData[5] = {
        -0.372718385753488, -0.1617981202790071, 0.01392301054676802, 
        -0.001380878979657776, 4.092875770669346
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
