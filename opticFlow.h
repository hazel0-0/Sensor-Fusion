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
#include "alphabot.h"
#include "pigpio.h"

using namespace cv;
using namespace std;

/*
struct OpticFlowParams {
    cv::Vec2d of;  
    std::vector<cv::Point2f> pts0;
    std::vector<cv::Point2f> pts1;
    float t;
};
*/
struct OpticFlowParams {
    cv::Vec2d &of;  
    std::vector<cv::Point2f> pts0;
    std::vector<cv::Point2f> pts1;
    float t;

    OpticFlowParams(cv::Vec2d &of_init, 
                    std::vector<cv::Point2f> pts0_init, 
                    std::vector<cv::Point2f> pts1_init, 
                    float t_init) 
        : of(of_init), pts0(pts0_init), pts1(pts1_init), t(t_init) {}
};


class OpticalFlowTracker
{
public:
    
    OpticalFlowTracker(int maxCorners = 100);
    ~OpticalFlowTracker();


    void start();
    void stop();
    void processFrame();
    void addFrame(const Mat& frame);
    void setOpticFlowCallback(std::function<void(OpticFlowParams)> callback);
    struct TimerData {
    static bool motorCondition;
    };
    TimerData data;


private:
/*
    static void timerCallback(void* userdata) {
        TimerData *data = static_cast<TimerData*>(userdata);
        data->motorCondition = !data->motorCondition;
    }
    */
    int maxCorners;
    std::vector<Scalar> colors;
    Mat old_gray;
    std::vector<Point2f> p0, p1;
    Mat mask;
    
    std::thread processingThread;
    std::atomic<bool> running;
    std::mutex frameMutex;
    std::condition_variable frameCondition;
    std::function<void(OpticFlowParams)> opticFlowCallback;
    Mat currentFrame;
    bool frameReady;
    AlphaBot alphabot;
    //OpticFlowParams params;
     
};

#endif // OPTICFLOW_H
