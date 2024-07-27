#include "calibration.h"
#include <iostream>

Calibration::Calibration(const std::vector<cv::Mat>& images, cv::Size boardSize, float squareSize)
    : images(images), boardSize(boardSize), squareSize(squareSize) {}

void Calibration::performCalibration() {
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints(1);

    for (int i = 0; i < boardSize.height; ++i) {
        for (int j = 0; j < boardSize.width; ++j) {
            objectPoints[0].push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }

    objectPoints.resize(images.size(), objectPoints[0]);

    for (const auto& image : images) {
        std::vector<cv::Point2f> pointBuf;
        bool found = cv::findChessboardCorners(image, boardSize, pointBuf,
                                               cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            cv::Mat viewGray;
            cv::cvtColor(image, viewGray, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(viewGray, pointBuf, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            imagePoints.push_back(pointBuf);
        }
    }

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(objectPoints, imagePoints, boardSize, cameraMatrix, distCoeffs, rvecs, tvecs);

    std::cout << "## Camera Calibration Results" << std::endl;
    std::cout << "* RMS error = " << rms << std::endl;
    std::cout << "* Camera matrix (K) = " << std::endl << "  " << cameraMatrix.row(0) << cameraMatrix.row(1) << cameraMatrix.row(2) << std::endl;
    std::cout << "* Distortion coefficient (k1, k2, p1, p2, k3, ...) = " << std::endl << "  " << distCoeffs.t() << std::endl;
}
