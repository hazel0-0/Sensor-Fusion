#ifndef OPTICFLOW_H
#define OPTICFLOW_H

#include <thread>
#include <iostream>
#include "libcam2opencv.h"
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

using namespace cv;
using namespace std;

class OpticalFlowTracker
{
public:
    OpticalFlowTracker(int maxCorners = 100);
    ~OpticalFlowTracker();
    void start();
    void stop();
    void processFrame();
    void addFrame(const Mat& frame);

private:
    
    int maxCorners;
    std::vector<Scalar> colors;
    Mat old_gray;
    std::vector<Point2f> p0, p1;
    Mat mask;
    
    std::thread processingThread;
    std::atomic<bool> running;
    std::mutex frameMutex;
    std::condition_variable frameCondition;
    Mat currentFrame;
    bool frameReady;
};

#endif // OPTICFLOW_H
