#include "window.h"
#include <iostream>

Window::Window() : calibrate(false) , showRectify(false), calibrator(), distortionCorrector(), tracker(){
    myCallback.window = this;
    camera.registerCallback(&myCallback);
    
    thermo = new QwtThermo;
    thermo->setFillBrush(QBrush(Qt::red));
    thermo->setScale(0, 255);
    thermo->show();

    image = new QLabel;

    calibrateButton = new QPushButton("Calibrate");
    rectifyButton = new QPushButton("Rectify");
    connect(calibrateButton, &QPushButton::clicked, this, &Window::onCalibrateButtonClicked);
    connect(rectifyButton, &QPushButton::clicked, this, &Window::onRectifyButtonClicked);

    hLayout = new QHBoxLayout();
    hLayout->addWidget(thermo);
    hLayout->addWidget(image);
    hLayout->addWidget(calibrateButton);
    hLayout->addWidget(rectifyButton);

    setLayout(hLayout);
    
    camera.start();
    tracker.start();
    
    distortionCorrector.setFrameCallback([this](const Mat& correctedFrame) {
        tracker.addFrame(correctedFrame);
        QImage qimg(correctedFrame.data, correctedFrame.cols, correctedFrame.rows, correctedFrame.step, QImage::Format_RGB888);
        image->setPixmap(QPixmap::fromImage(qimg.rgbSwapped()));

        const int h = qimg.height();
        const int w = qimg.width();
        const QColor c = qimg.pixelColor(w / 2, h / 2);
        thermo->setValue(c.lightness());
        update();
    });
}

Window::~Window() {
    camera.stop();
    calibrator.stop();
    tracker.stop();
    distortionCorrector.stop();
}


void Window::updateImage(const cv::Mat &mat) {
    if (calibrate) 
    {
    std::vector<cv::Point2f> pts;
    bool found = cv::findChessboardCorners(mat, cv::Size(9,6), pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    
    if (found) {
	calibrator.addFrame(mat);
        cv::drawChessboardCorners(mat, cv::Size(9,6),  pts, found);
    }
    }
    if (showRectify) {
        distortionCorrector.addFrame(mat);
    }
    else
    {
   // tracker.addFrame(mat); // Process the frame with OpticalFlowTracker
    
    const QImage frame(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
    image->setPixmap(QPixmap::fromImage(frame));
    const int h = frame.height();
    const int w = frame.width();
    const QColor c = frame.pixelColor(w / 2, h / 2);
    thermo->setValue(c.lightness());
    update();
    }

}


void Window::onCalibrateButtonClicked() {
    calibrate = !calibrate;
    if(calibrate == true) calibrator.start();
    if(calibrate == false) calibrator.stopCollection();
    calibrateButton->setText(calibrate ? "Stop Calibrate" : "Calibrate");
    
}
void Window::onRectifyButtonClicked()
{
    showRectify = !showRectify;
    if(showRectify == true) distortionCorrector.start();
    if(showRectify == false) distortionCorrector.stop();
    rectifyButton->setText(showRectify ? "Stop Rectify" : "Rectify");
}

