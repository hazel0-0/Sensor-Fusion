#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <vector>
#include "libcam2opencv.h"

class Calibration {
public:
    Calibration(const std::vector<cv::Mat>& images, cv::Size boardSize, float squareSize);

    void performCalibration();

private:
    std::vector<cv::Mat> images;
    cv::Size boardSize;
    float squareSize;
};

#endif // CALIBRATION_H
