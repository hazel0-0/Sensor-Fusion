#include "opticFlow.h"

OpticalFlowTracker::OpticalFlowTracker(int maxCorners)
    : maxCorners(maxCorners), running(false), frameReady(false)
{
    // Create some random colors
    RNG rng;
    for (int i = 0; i < maxCorners; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(Scalar(r, g, b));
    }
}


OpticalFlowTracker::~OpticalFlowTracker()
{
    stop();
}
void OpticalFlowTracker::start()
{
    running = true;
    processingThread = std::thread(&OpticalFlowTracker::processFrame, this);
     std::cout << "Tracker start"<< std::endl;;
    
}

void OpticalFlowTracker::stop()
{
    running = false;
    frameCondition.notify_all();
    if (processingThread.joinable())
    {
        processingThread.join();
    }
}
void OpticalFlowTracker::addFrame(const Mat& frame)
{
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        frame.copyTo(currentFrame);
        frameReady = true;
        std::cout << "Frameadded"<< std::endl;;
    }
    frameCondition.notify_one();
}

void OpticalFlowTracker::setOpticFlowCallback(std::function<void(const std::vector<Point2f>)> callback)
{
    opticFlowCallback = callback;
}

void OpticalFlowTracker::processFrame()
{

    while (running)
    {
    Mat frame;
        {
            std::unique_lock<std::mutex> lock(frameMutex);
            frameCondition.wait(lock, [this] { return frameReady || !running; });
            if (!running)
                break;
            frameReady = false;
            frame = currentFrame.clone();
            
        }
    if (frame.empty()) continue;

    Mat frame_gray;
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

    // Initialize the first frame and mask if not yet done
    if (old_gray.empty())
    {
        old_gray = frame_gray.clone();
        goodFeaturesToTrack(old_gray, p0, maxCorners, 0.3, 7, Mat(), 7, false, 0.04);
        mask = Mat::zeros(frame.size(), frame.type());
        continue;
    }


    // calculate optical flow
    std::vector<uchar> status;
    std::vector<float> err;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
    calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(15, 15), 2, criteria);
    
    std::cout << "Frameaccepted"<< std::endl;;
    std::vector<Point2f> good_new;
    for (size_t i = 0; i < p0.size(); i++)
    {
        // Select good points
        if (status[i] == 1)
        {
            good_new.push_back(p1[i]);
            // draw the tracks
            line(mask, p1[i], p0[i], colors[i], 2);
            circle(frame, p1[i], 5, colors[i], -1);


            // Print optical flow data
            optic_Flow[i].x = p1[i].x - p0[i].x;
            optic_Flow[i].y = p1[i].y - p0[i].y;
            cout << "Old position: (" << p0[i].x << ", " << p0[i].y << ")";
            cout << " New position: (" << p1[i].x << ", " << p1[i].y << ")";
            cout << " Flow vector: (" << optic_Flow[i].x << ", " << optic_Flow[i].y<< ")" << endl;
        }
        }
        add(frame, mask, frame);

        old_gray = frame_gray.clone();
        p0 = good_new;

        imshow("Optical Flow", frame);
        waitKey(1);
    }
    opticFlowCallback(optic_Flow);
}

