
#include "libcam2opencv.h"
#include "iostream"

int main()
{
    const char* video_file = "/home/haz/Desktop/SensorFusion/libcamera2opencv-master/qtviewer/video/output_0.avi";
    cv::Size board_pattern(9, 6);
    float board_cellsize = 0.025f;
    bool select_images = true;

    // Open a video
    cv::VideoCapture video;
    if (!video.open(video_file)) return -1;

    // Select images
    std::vector<cv::Mat> images;
    while (true)
    {
        // Grab an image from the video
        cv::Mat image;
        video >> image;
        if (image.empty()) break;

        if (select_images)
        {
            
            // Show the image and keep it if selected
            cv::imshow("Camera Calibration", image);
            int key = cv::waitKey(1);
            if (key == 32)                              // Space: Pause and show corners
            {
                cv::Mat gray;
                cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

                int grayKey = cv::waitKey();
                if (grayKey == 27) break; // 按 ESC 键退出
                std::vector<cv::Point2f> pts;
                bool complete = cv::findChessboardCorners(gray, board_pattern, pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
                std::cout << "findChessboardCorners complete: " << complete << std::endl;
                cv::Mat display = image.clone();
                cv::drawChessboardCorners(display, board_pattern, pts, complete);
                cv::imshow("Camera Calibration", display);
                key = cv::waitKey();
                if (key == 13 && complete) images.push_back(image); // Enter: Select the image
            }
            if (key == 27) break;                       // ESC: Exit (Complete image selection)
        }
        else images.push_back(image);
    }
    video.release();
    if (images.empty()) {
        std::cerr << "No images selected" << std::endl;
        return -1;
     }
    // Find 2D corner points from the given images
    std::vector<std::vector<cv::Point2f>> img_points;
    for (size_t i = 0; i < images.size(); i++)
    {
        cv::Mat gray;
        cv::cvtColor(images[i], gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> pts;
        //if (cv::findChessboardCorners(images[i], board_pattern, pts))
         //   img_points.push_back(pts);
         bool found = cv::findChessboardCorners(gray, board_pattern, pts, cv::CALIB_CB_ADAPTIVE_THRESH);
        if (found) {
            std::cout << "Corners found in image " << i << std::endl;
            img_points.push_back(pts);
        } else {
            std::cout << "Corners not found in image " << i << std::endl;
        }
    }
    if (img_points.empty()) 
    {
        std::cerr << "No corner points found" << std::endl;
        return -1;
    }

    // Prepare 3D points of the chess board
    std::vector<std::vector<cv::Point3f>> obj_points(1);
    for (int r = 0; r < board_pattern.height; r++)
        for (int c = 0; c < board_pattern.width; c++)
            obj_points[0].push_back(cv::Point3f(board_cellsize * c, board_cellsize * r, 0));
    obj_points.resize(img_points.size(), obj_points[0]); // Copy

    // Calibrate the camera
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist_coeff = cv::Mat::zeros(4, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(obj_points, img_points, images[0].size(), K, dist_coeff, rvecs, tvecs);

    // Print calibration results
    std::cout << "## Camera Calibration Results" << std::endl;
    std::cout << "* The number of applied images = " << img_points.size() << std::endl;
    std::cout << "* RMS error = " << rms << std::endl;
    std::cout << "* Camera matrix (K) = " << std::endl << "  " << K.row(0) << K.row(1) << K.row(2) << std::endl;
    std::cout << "* Distortion coefficient (k1, k2, p1, p2, k3, ...) = " << std::endl << "  " << dist_coeff.t() << std::endl;
    return 0;
}
