#include "distortionCorrection.h"

DistortionCorrection::DistortionCorrection()  : cameraMatrix(CameraParameters::cameraMatrixData),
      distCoeffs(Mat(1, 5, CV_64F, (void*)CameraParameters::distCoeffsData)), 
      running(false), frameReady(false)
{


}

DistortionCorrection::~DistortionCorrection()
{
    stop();
}

void DistortionCorrection::start()
{
    running = true;
    processingThread = std::thread(&DistortionCorrection::process, this);
}

void DistortionCorrection::stop()
{
    running = false;
    frameCondition.notify_all();
    if (processingThread.joinable())
    {
        processingThread.join();
    }
}

void DistortionCorrection::addFrame(const Mat& frame)
{
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        frame.copyTo(currentFrame);
        frameReady = true;
    }
    frameCondition.notify_one();
}

void DistortionCorrection::setFrameCallback(std::function<void(const Mat&)> callback)
{
    frameCallback = callback;
}

void DistortionCorrection::process()
{
    while (running)
    {
        Mat frame;
        {
            std::unique_lock<std::mutex> lock(frameMutex);
            frameCondition.wait(lock, [this] { return frameReady || !running; });
            if (!running)
                break;
            frameReady = false;
            frame = currentFrame.clone();
        }

        if (frame.empty())
            continue;

        Mat correctedFrame;
        if (map1.empty() || map2.empty())
        {
            initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), Mat(), frame.size(), CV_32FC1, map1, map2);
        }
        remap(frame, correctedFrame, map1, map2, INTER_LINEAR);
        putText(correctedFrame, "Rectified", Point(5, 15), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));

        if (frameCallback)
        {
            frameCallback(correctedFrame);
        }
    }
}



