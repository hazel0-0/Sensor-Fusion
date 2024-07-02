#ifndef WINDOW_H
#define WINDOW_H

#include <qwt/qwt_thermo.h>

#include <QBoxLayout>
#include <QPushButton>
#include <QLabel>

#include "libcam2opencv.h"

// class definition 'Window'
class Window : public QWidget
{
    // must include the Q_OBJECT macro for for the Qt signals/slots framework to work with this class
    Q_OBJECT

public:
    Window();
    ~Window();

    void updateImage(const cv::Mat &mat);

    
    QwtThermo    *thermo;
    QHBoxLayout  *hLayout;  // horizontal layout
    QLabel       *image;

    struct MyCallback : Libcam2OpenCV::Callback {
	Window* window = nullptr;
	virtual void hasFrame(const cv::Mat &frame, const libcamera::ControlList &) {
	    if (nullptr != window) {
		window->updateImage(frame);
	    }

	}
    };

    Libcam2OpenCV camera;
    MyCallback myCallback;
private:
    void startRecording();
    void stopRecording();

    cv::VideoWriter *videoWriter; //
    bool recording;
    std::string videoFilePath;
    int counter = 0;
};

#endif // WINDOW_H
