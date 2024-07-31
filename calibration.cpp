#include "calibration.h"
#include <iostream>

Calibration::Calibration()
    : running(false), frameReady(false), 
    boardSize(9, 6), squareSize(1.8) {}

Calibration::~Calibration() {
    stop();
}

void Calibration::start() {
    running = true;
    calibrationThread = std::thread(&Calibration::processImages, this);
}

void Calibration::stop() {
    frameCondition.notify_all();
    if (calibrationThread.joinable()) {
        calibrationThread.join();
    }
}

void Calibration::addFrame(const cv::Mat& frame) {
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        frame.copyTo(currentFrame);
        frameReady = true;
    }
    frameCondition.notify_one();
}


void Calibration::stopCollection(){
    running = false;
}

void Calibration::setFrameCallback(std::function<void(const cv::Mat&)> callback)
{
    frameCallback = callback;
}


void Calibration::processImages() {
    while (running) {
        //callback();
        cv::Mat image;
        {
            std::unique_lock<std::mutex> lock(frameMutex);
            frameCondition.wait(lock, [this] { return frameReady || !running; });
            if (!running)
                break;
            frameReady = false;

            image = currentFrame.clone();
        }
        
        if (image.empty()) 
        continue;

        std::vector<cv::Point2f> pointBuf;
        bool found = cv::findChessboardCorners(image, boardSize, pointBuf,
                                               cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
                 
            cv::drawChessboardCorners(image, cv::Size(9,6),  pointBuf, found);
            if (frameCallback)
            {
                 frameCallback(image);
            }            
            cv::Mat viewGray;
            cv::cvtColor(image, viewGray, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(viewGray, pointBuf, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            {
                std::lock_guard<std::mutex> lock(mtx);
                imagePoints.push_back(pointBuf);
            }
            
        }
        else frameCallback(image);
        
    }   
    performCalibration();
    stop();

}

void Calibration::performCalibration() {
    std::lock_guard<std::mutex> lock(mtx);
    if (imagePoints.empty()) {
        std::cout << "No corners found in any image. Calibration cannot proceed." << std::endl;
        return;
    }

    std::vector<std::vector<cv::Point3f>> objectPoints(1);
    for (int i = 0; i < boardSize.height; ++i) {
        for (int j = 0; j < boardSize.width; ++j) {
            objectPoints[0].push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }
    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(objectPoints, imagePoints, boardSize, cameraMatrix, distCoeffs, rvecs, tvecs);

    std::cout << "## Camera Calibration Results" << std::endl;
    std::cout << "* RMS error = " << rms << std::endl;
    std::cout << "* Camera matrix (K) = " << std::endl << "  " << cameraMatrix.row(0) << cameraMatrix.row(1) << cameraMatrix.row(2) << std::endl;
    std::cout << "* Distortion coefficient (k1, k2, p1, p2, k3, ...) = " << std::endl << "  " << distCoeffs.t() << std::endl;
}
