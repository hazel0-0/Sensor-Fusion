#include "window.h"

Window::Window()
{

	myCallback.window = this;
	camera.registerCallback(&myCallback);
	
	// set up the thermometer
	thermo = new QwtThermo; 
	thermo->setFillBrush( QBrush(Qt::red) );
	thermo->setScale(0, 255);
	thermo->show();

	image = new QLabel;
	//recording buttons
        QPushButton *startButton = new QPushButton("Start Recording");
        QPushButton *stopButton = new QPushButton("Stop Recording");
        connect(startButton, &QPushButton::clicked, this, &Window::startRecording);
        connect(stopButton, &QPushButton::clicked, this, &Window::stopRecording);
	// plot to the left of button and thermometer
	hLayout = new QHBoxLayout();
	hLayout->addWidget(thermo);
	hLayout->addWidget(image);
	hLayout->addWidget(startButton);
        hLayout->addWidget(stopButton);

	setLayout(hLayout);
	camera.start();
	
	videoWriter = nullptr; //
        recording = false;
}

Window::~Window()
{
	camera.stop();
    if (videoWriter && videoWriter->isOpened()) {
        videoWriter->release();
        delete videoWriter;
    }
}

void Window::startRecording()
{
    if (videoWriter && videoWriter->isOpened()) {
        videoWriter->release();
        delete videoWriter;
    }
    videoFilePath = "/home/haz/Desktop/SensorFusion/libcamera2opencv-master/qtviewer/video/output_" + std::to_string(counter) + ".avi";

    recording = true;
    counter++;
}

// 停止录制视频
void Window::stopRecording()
{
    if (videoWriter && videoWriter->isOpened()) {
        videoWriter->release();
        delete videoWriter;
        videoWriter = nullptr;
    }
    recording = false;
}

void Window::updateImage(const cv::Mat &mat) {
	const QImage frame(mat.data, mat.cols, mat.rows, mat.step,
			   QImage::Format_RGB888);
	image->setPixmap(QPixmap::fromImage(frame));
	const int h = frame.height();
	const int w = frame.width();
	const QColor c = frame.pixelColor(w/2, h/2);
	thermo->setValue(c.lightness());
	update();
	
	if (recording && !videoWriter) {
        videoWriter = new cv::VideoWriter(videoFilePath,
                                          cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                          10, cv::Size(w,h), true);

            if (!videoWriter->isOpened()) {
                 std::cerr << "Failed to open video writer" << std::endl;
                 recording = false;
               }
        }
	
	if (recording && videoWriter && videoWriter->isOpened()) {
             videoWriter->write(mat);
        }
}
