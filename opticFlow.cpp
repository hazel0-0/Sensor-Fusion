#include "opticFlow.h"
#include "math.h"

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
    
}

void OpticalFlowTracker::stop()
{

    running = false;
    frameCondition.notify_all();
    if (processingThread.joinable())
    {
        processingThread.join();
    }
    old_gray.release();
    p0.clear();
    p1.clear();

}
void OpticalFlowTracker::addFrame(const Mat& frame)
{
   // static int frameCounter = 0;  // A static counter that tracks the number of frames received

    {
        std::lock_guard<std::mutex> lock(frameMutex);
        //frameCounter++;
        //if (frameCounter == 1) {
            frame.copyTo(currentFrame);
            frameReady = true;
          //  frameCounter = 0;  
            frameCondition.notify_one();  // notify condition variable
        //} 
    }
}

void OpticalFlowTracker::setOpticFlowCallback(std::function<void(OpticFlowParams)> callback)
{
    opticFlowCallback = callback;
}



void OpticalFlowTracker::processFrame()
{
    auto previousTime = std::chrono::high_resolution_clock::now();

    while (running)
    {
    Mat frame;
    static bool ptschange = true;
    static cv::Vec2d signal(0, 0);
        {
            std::unique_lock<std::mutex> lock(frameMutex);
            frameCondition.wait(lock, [this] { return frameReady || !running; });
            if (!running)
                break;
            frameReady = false;
            frame = currentFrame.clone();
        }
    
    if (frame.empty()) continue;
    
    float frameTime;
    Mat frame_gray;
    static int k;
    cv::Vec2d optic_Flow(0, 0);
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

    // Initialize the first frame and mask if not yet done
    if (old_gray.empty())
    {
        
        old_gray = frame_gray.clone();

        // Create a mask that only allows feature points to be selected within a specific area
        mask = Mat::zeros(old_gray.size(), CV_8UC1);
        signal[0] = 0;
        signal[1] = 0;

        // Feature points are selected only in a rectangular area in the center of the image
        int x = old_gray.cols / 3;
        int y = old_gray.rows / 3;
        int width = old_gray.cols / 3;
        int height = old_gray.rows / 3;
        rectangle(mask, Rect(x, y, width, height), Scalar(255), FILLED);
        goodFeaturesToTrack(old_gray, p0, maxCorners, 0.3, 7, mask, 7, false, 0.04);

        k = p0.size();
        //cout << "Number of points: " << p0.size() << endl;
        mask = Mat::zeros(frame.size(), frame.type());
        continue;
    }
    
    

    // calculate optical flow
    std::vector<uchar> status;
    std::vector<float> err;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
    //cout << "Number of points: " << p0.size() << endl;
    if (p0.size()< k/2 | p0.size()==0) {
        old_gray.release();//track new good Features
        continue;
    }
    
    
    calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(15, 15), 2, criteria);
   /* 
    if (ptschange)
        {
            if (params.pts0 != p0)
            {
            ptschange = false;
            std::cout<<"point_0 get"<<std::endl;
            params.pts0 = p0;
            }
        }
*/
    std::vector<Point2f> good_new;
    
    if (p0.size() == p1.size()) {
        for (size_t i = 0; i < p0.size(); i++){
            // Select good points
            if (status[i] == 1){
                good_new.push_back(p1[i]);
               //draw the tracks
                 //line(mask, p1[i], p0[i], colors[i], 2);
                 //circle(frame, p1[i], 5, colors[i], -1);
          
                optic_Flow[0] += (p1[i].x - p0[i].x);
                optic_Flow[1] += (p1[i].y - p0[i].y);
            }
        }
    }

       // add(frame, mask, frame);
        old_gray = frame_gray.clone();
        
                
        if (!good_new.empty()) {

            optic_Flow[0] /= static_cast<double>(good_new.size());
            signal[0] += optic_Flow[0];
            optic_Flow[1] /= static_cast<double>(good_new.size());
            signal[1] += optic_Flow[1];
         }
        // cout << " Average Flow vector: " << optic_Flow[0] << ", " << optic_Flow[1] << endl;
        // cout << " Average signal: " << signal[0] << ", " << signal[1] << endl;

       if (opticFlowCallback)  
       {
           auto currentTime = std::chrono::high_resolution_clock::now();
           std::chrono::duration<double> timeSpan = currentTime - previousTime;//Time between frames
           float timeElapsed= timeSpan.count(); 
           //params.of = optic_Flow;
           //params.pts1 = {}; // set empty vector to avoid affinetrans happened to every opticFlow vector
           OpticFlowParams params(optic_Flow, p0, p1, timeElapsed);
           opticFlowCallback(params);
           previousTime = currentTime;
       }
        
        if(abs(signal[0]) > 100) {
        
        old_gray.release();//release old_gray to track new good Features
  /*
        ptschange = true;   
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> timeSpan = currentTime - previousTime;//Time between frames
        std::cout<<"the signal:"<< signal <<std::endl;
        params.of=signal;
        params.t = timeSpan.count(); 
        params.pts1 = p1;
        opticFlowCallback(params);
        previousTime = currentTime;
    }
    //when the robot first have a horizontal migration
    else if (ptschange) {
        if (abs(signal[0]) > 1){
            previousTime = std::chrono::high_resolution_clock::now();
            params.pts0 = p0;
            std::cout<<"horizontal migration happened"<< signal <<std::endl;
            ptschange = false;
        } */
    }
   
        // imshow("Optical Flow", frame);
        waitKey(1);
        p0 = good_new;
        

    }
}

