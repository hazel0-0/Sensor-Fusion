#include "window.h"
#include <iostream>

Window::Window() : calibrate(false) , record(false), showRectify(false), calibrator(), distortionCorrector(), tracker(){
    myCallback.window = this;
    camera.registerCallback(&myCallback);
    
    thermo = new QwtThermo;
    thermo->setFillBrush(QBrush(Qt::red));
    thermo->setScale(0, 255);
    thermo->show();

    image = new QLabel;

    calibrateButton = new QPushButton("Calibrate");
    recordButton = new QPushButton("Record");
    rectifyButton = new QPushButton("Rectify");
    connect(calibrateButton, &QPushButton::clicked, this, &Window::onCalibrateButtonClicked);
    connect(recordButton, &QPushButton::clicked, this, &Window::onRecordButtonClicked);
    connect(rectifyButton, &QPushButton::clicked, this, &Window::onRectifyButtonClicked);

    hLayout = new QHBoxLayout();
    hLayout->addWidget(thermo);
    hLayout->addWidget(image);
    hLayout->addWidget(calibrateButton);
    hLayout->addWidget(recordButton);
    hLayout->addWidget(rectifyButton);

    setLayout(hLayout);
    
    camera.start();
    tracker.start();
    
    distortionCorrector.setFrameCallback([this](const cv::Mat& correctedFrame) {
       // tracker.addFrame(correctedFrame);
        display(correctedFrame);
	record = false;
	recordButton->setText(record ? "Stop record" : "record");

    });
    calibrator.setFrameCallback([this](const cv::Mat& Frame) {
	display(Frame);

    });
}

Window::~Window() {
    camera.stop();
    calibrator.stop();
    tracker.stop();
    distortionCorrector.stop();
}


void Window::updateImage(const cv::Mat &mat) {
    
    if (record) 
    {
	calibrator.addFrame(mat);
    }
    
    else if (showRectify) 
    {
        distortionCorrector.addFrame(mat);
    }
    
    else
    {
   // tracker.addFrame(mat); // Process the frame with OpticalFlowTracker
     display(mat);
    }

}

void Window::display(const cv::Mat &mat) {
    
    
    const QImage frame(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
    image->setPixmap(QPixmap::fromImage(frame));
    const int h = frame.height();
    const int w = frame.width();
    const QColor c = frame.pixelColor(w / 2, h / 2);
    thermo->setValue(c.lightness());
    update();
    
    }

void Window::onRecordButtonClicked()
{
    record =!record;
    recordButton->setText(record ? "Stop record" : "record");
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

