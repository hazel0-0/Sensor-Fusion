#ifndef WINDOW_H
#define WINDOW_H

#include <qwt/qwt_thermo.h>
#include <QBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QWidget>
#include "libcam2opencv.h"
#include "calibration.h"


class Window : public QWidget
{
    Q_OBJECT

public:
    Window();
    ~Window();

    void updateImage(const cv::Mat &mat);
    void saveImage(const cv::Mat &mat);
    void Recording();


private:
    QwtThermo    *thermo;
    QHBoxLayout  *hLayout;
    QLabel       *image;
    bool recording;
    std::vector<cv::Mat> savedImages;
    QPushButton *calibrateButton;
    QPushButton *recordButton;

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
    float board_cellsize = 0.0157f;
    int counter = 0;

private:
    void onCalibrateButtonClicked();
};


#endif // WINDOW_H

