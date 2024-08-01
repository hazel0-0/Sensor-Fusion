#ifndef WINDOW_H
#define WINDOW_H

#include <qwt/qwt_thermo.h>
#include <QBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QWidget>
#include "libcam2opencv.h"
#include "calibration.h"
#include "opticFlow.h"
#include "distortionCorrection.h"



class Window : public QWidget
{
    Q_OBJECT

public:
    Window();
    ~Window();

    void updateImage(const cv::Mat &mat);
    void display(const cv::Mat &mat);


private:
    QwtThermo    *thermo;
    QHBoxLayout  *hLayout;
    QLabel       *image;
    bool calibrate;
    bool showRectify;
    bool record;
    bool showOpticFlow;
    
    QPushButton *calibrateButton;
    QPushButton *rectifyButton;
    QPushButton *recordButton;
    QPushButton *opticButton;

    struct MyCallback : Libcam2OpenCV::Callback {
        Window* window = nullptr;
        virtual void hasFrame(const cv::Mat &frame, const libcamera::ControlList &) {
            if (window) {
                window->updateImage(frame);
            }
        }
    };


    Libcam2OpenCV camera;
    MyCallback myCallback;
    int counter = 0;
    
    Calibration calibrator;
    OpticalFlowTracker tracker; // Add the OpticalFlowTracker instance
    DistortionCorrection distortionCorrector; // Add the DistortionCorrection instance

private:
    void onCalibrateButtonClicked();
    void onRecordButtonClicked();
    void onRectifyButtonClicked();
    void onopticButtonClicked();
    
};


#endif // WINDOW_H

