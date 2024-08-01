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

      1333.984718906881, 0, 400.8083055432996,
      0, 1331.538365153853, 300.8161085189564,
      0, 0, 1

    };
    static constexpr double distCoeffsData[5] = {
    -0.4340982675228632, 0.2992928558391148, -0.0002859856389761491, -0.001749866928315733, 1.048366966225898
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
