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
#include "navigation.h"

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
    QVBoxLayout  *vLayout;
    QLabel       *image;
    bool calibrate;
    bool showRectify;
    bool record;
    bool showOpticFlow;
    
    QPushButton *calibrateButton;
    QPushButton *rectifyButton;
    QPushButton *recordButton;
    QPushButton *opticButton;
    QPushButton *startBotButton; 

    struct MyCallback : Libcam2OpenCV::Callback {
        Window* window = nullptr;
        virtual void hasFrame(const cv::Mat &frame, const libcamera::ControlList &) {
            if (window) {
                window->updateImage(frame);
            }
        }
    };
    
    void timerEvent();

	static void timerCallback(void* alphabotptr) {
		((Window*)alphabotptr)->timerEvent();
	}


    Libcam2OpenCV camera;
    MyCallback myCallback;

    
    Calibration calibrator;
    OpticalFlowTracker tracker; // Add the OpticalFlowTracker instance
    DistortionCorrection distortionCorrector; // Add the DistortionCorrection instance
    AngleNavigation* angle;
    AlphaBot* alphabot;

private:
    void onCalibrateButtonClicked();
    void onRecordButtonClicked();
    void onRectifyButtonClicked();
    void onopticButtonClicked();
    void onStartBotButtonClicked();
};


#endif // WINDOW_H

