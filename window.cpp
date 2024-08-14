#include "window.h"
#include <iostream>
#include <unistd.h>

Window::Window() : calibrate(false) , record(false), showRectify(false),showOpticFlow(false), calibrator(), distortionCorrector(), tracker(){
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
    opticButton = new QPushButton("OpticFlow");
    startBotButton = new QPushButton("Start Bot"); 
    
    connect(calibrateButton, &QPushButton::clicked, this, &Window::onCalibrateButtonClicked);
    connect(recordButton, &QPushButton::clicked, this, &Window::onRecordButtonClicked);
    connect(rectifyButton, &QPushButton::clicked, this, &Window::onRectifyButtonClicked);
    connect(opticButton, &QPushButton::clicked, this, &Window::onopticButtonClicked);
    connect(startBotButton, &QPushButton::clicked, this, &Window::onStartBotButtonClicked);
    

    
    vLayout = new QVBoxLayout();
    vLayout->addWidget(calibrateButton);
    vLayout->addWidget(recordButton);
    vLayout->addWidget(rectifyButton);
    vLayout->addWidget(opticButton);
    vLayout->addWidget(startBotButton);
    
    hLayout = new QHBoxLayout();
    hLayout->addWidget(thermo);
    hLayout->addWidget(image);
    hLayout->addLayout(vLayout);

    setLayout(hLayout);
    
    camera.start();
    angle = new AngleNavigation();
    alphabot = new AlphaBot();
    affinetrans = new AffineTrans();

    tracker.setOpticFlowCallback([this](const OpticFlowParams& params) {
        angle->filter(params.of);

        if (!params.pts1.empty())
        {affinetrans->addTask(params.of, params.pts0, params.pts1, params.t);
            }

    });


    distortionCorrector.setFrameCallback([this](const cv::Mat& correctedFrame) {
        display(correctedFrame);

    });
    calibrator.setFrameCallback([this](const cv::Mat& Frame, bool ptsfound) {
	static int frameCounter = 0;
	display(Frame);
	if (ptsfound)frameCounter++;
    if (frameCounter == 5) 
	{record = false; frameCounter=0; recordButton->setText(record ? "Stop record" : "record");}
    });
}

Window::~Window() {
    camera.stop();
    calibrator.stop();
    tracker.stop();
    distortionCorrector.stop();
    angle->stop();
    gpioSetTimerFuncEx(0,1000,NULL,(void*)this);
    alphabot->stop();
    affinetrans->stop();
}

void Window::updateImage(const cv::Mat &mat) {
    
    
    if (record) {
	    calibrator.addFrame(mat);
    }
    
    else if (showRectify) {
        distortionCorrector.addFrame(mat);
    }
    
    else{
        display(mat);
    }

}

void Window::display(const cv::Mat &mat) {
    if(showOpticFlow) tracker.addFrame(mat); // Process the frame with OpticalFlowTracker
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
void Window::onopticButtonClicked()
{
    showOpticFlow = !showOpticFlow;
    if(showOpticFlow == true) {
        tracker.start(); 
        affinetrans->start(); 
        angle->start(); 
        }
    if(showOpticFlow == false) {
        tracker.stop(); 
        affinetrans->stop(); 
        angle->stop(); 
        }
    opticButton->setText(showOpticFlow ? "Stop" : "OpticFlow");
}

void Window::onStartBotButtonClicked() {
    
    // Start AlphaBot and set right wheel speed 
    alphabot->start(); 
    //alphabot->setRightWheelSpeed(0.2f);
    
    // turning control test
     angle->angleControl(90.0);
    //gpioSetTimerFuncEx(0, 3000, timerCallback, (void*)this);
}



void Window::timerEvent()
{

}

