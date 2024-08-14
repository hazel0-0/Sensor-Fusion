#ifndef AFFINETRANS_H
#define AFFINETRANS_H

#include "opticFlow.h"
#include <vector>
#include <box2d/box2d.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

class AffineTrans {
public:
    AffineTrans();
    ~AffineTrans();

    void start();
    void stop();
    
    // Adds a task for processing optical flow data and current points
    void addTask(const cv::Vec2d opticFlow, std::vector<Point2f> current, std::vector<Point2f> previous, float timeElapsed);
    
    // Gets the result of the affine transformation
    b2Transform getResult();

private:
    // Structure to represent a task in the queue
    struct Task {
        cv::Vec2d opticFlow;
        std::vector<Point2f> current;
        std::vector<Point2f> previous;
        float timeElapsed;
    };

    // The thread function that processes tasks
    void processTasks();
    
    // The function that performs the affine transformation estimate
    void affineTransEstimate(const cv::Vec2d opticFlow, std::vector<Point2f> current, std::vector<Point2f> previous, float timeElapsed);
    
    // Variables for thread management
    std::thread workerThread;
    std::mutex mtx;
    std::condition_variable cv;
    
    // Flag to control the running state of the worker thread
    bool running;
    
    // Flag to indicate if the current operation is complete
    bool done;

    // Flag to indicate if the system is ready
    bool ready;

    // Queue to hold tasks that need to be processed
    std::queue<Task> taskQueue;
    
    // The result of the affine transformation
    b2Transform result;
};


#endif
