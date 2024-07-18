#include "libcam2opencv.h"
#include "iostream"
int main()
{
    // The given video and calibration data
    const char* video_file = "/home/haz/Desktop/SensorFusion/libcamera2opencv-master/qtviewer/video/output_0.avi";
    cv::Matx33d K(428.5051706024782, 0, 413.8113420244575,
                  0, 424.1431678503812, 388.0143783482946,
                  0, 0, 1); // Derived from `calibrate_camera.py`
    std::vector<double> dist_coeff = { -0.002555073027858495, 0.09067783475503255, 0.001934474355701299, 0.004440005456591022, -0.08295245862517196 };

    // Open a video
    cv::VideoCapture video;
    if (!video.open(video_file)) return -1;

    // Run distortion correction
    bool show_rectify = true;
    cv::Mat map1, map2;
    while (true)
    {
        // Read an image from the video
        cv::Mat image;
        video >> image;
        if (image.empty()) break;

        // Rectify geometric distortion (Alternative: `cv.undistort()`)
        cv::String info = "Original";
        if (show_rectify)
        {
            if (map1.empty() || map2.empty())
                cv::initUndistortRectifyMap(K, dist_coeff, cv::Mat(), cv::Mat(), image.size(), CV_32FC1, map1, map2);
            cv::remap(image, image, map1, map2, cv::InterpolationFlags::INTER_LINEAR);
            info = "Rectified";
        }
        cv::putText(image, info, cv::Point(5, 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Vec3b(0, 255, 0));

        // Show the image
        cv::imshow("Geometric Distortion Correction", image);
        int key = cv::waitKey(10);
        if (key == 32) key = cv::waitKey();                 // Space: Pause
        if (key == 27) break;                               // ESC: Exit
        else if (key == 9) show_rectify = !show_rectify;    // Tab: Toggle the mode
    }

    video.release();
    return 0;
}
