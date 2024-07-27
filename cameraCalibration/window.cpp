#include "window.h"
#include <iostream>

Window::Window() : recording(false) {
    myCallback.window = this;
    camera.registerCallback(&myCallback);

    thermo = new QwtThermo;
    thermo->setFillBrush(QBrush(Qt::red));
    thermo->setScale(0, 255);
    thermo->show();

    image = new QLabel;

    calibrateButton = new QPushButton("Calibrate");
    recordButton = new QPushButton("Recording");
    connect(recordButton, &QPushButton::clicked, this, &Window::Recording);
    connect(calibrateButton, &QPushButton::clicked, this, &Window::onCalibrateButtonClicked);
    hLayout = new QHBoxLayout();
    hLayout->addWidget(thermo);
    hLayout->addWidget(image);
    hLayout->addWidget(recordButton);
    hLayout->addWidget(calibrateButton);

    setLayout(hLayout);
    camera.start();
}

Window::~Window() {
    camera.stop();
}

void Window::Recording() {
    recording = true;
}

void Window::updateImage(const cv::Mat &mat) {
    if (recording) 
    {
    cv::Size board_pattern(9, 6);
    std::vector<cv::Point2f> pts;
    bool found = cv::findChessboardCorners(mat, board_pattern, pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    
    if (found) {
	saveImage(mat);
        cv::drawChessboardCorners(mat, board_pattern,  pts, found);
	recording = false;
    }
    }
    const QImage frame(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
    image->setPixmap(QPixmap::fromImage(frame));
    const int h = frame.height();
    const int w = frame.width();
    const QColor c = frame.pixelColor(w / 2, h / 2);
    thermo->setValue(c.lightness());
    update();

}

void Window::saveImage(const cv::Mat &mat) {
    
    savedImages.push_back(mat.clone());
    std::cout << "Image saved, total: " << savedImages.size() << std::endl;
}



void Window::onCalibrateButtonClicked() {
    if (!savedImages.empty()) {
	Calibration calib(savedImages, cv::Size(9, 6), board_cellsize);
        calib.performCalibration();
    } else {
        std::cout << "No images to calibrate." << std::endl;
    }

}


