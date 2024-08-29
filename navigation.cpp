#include "navigation.h"
#include <cmath>

AngleNavigation::AngleNavigation() : running(false), frameReady(false), motorCondition(false), alphabot(new AlphaBot()) {
    low_pass.setup(FPS, cutoff_frequency);
    band_stop.setup(FPS, DC, band_width);
}

AngleNavigation::~AngleNavigation()
{
    stop();
    delete alphabot;  // Make sure to delete the allocated alphabot
}


void AngleNavigation::start() {
    if (!running) {
        running = true;
        filteringThread = std::thread(&AngleNavigation::processFilter, this);
    }
}

void AngleNavigation::stop() {
    if (running) {
        running = false;
        current_angle = 0;
        frameCondition.notify_all();
        if (filteringThread.joinable()) {
            filteringThread.join();
        }
    }
}

void AngleNavigation::filter(const cv::Vec2d& optic_flow, const float timeElapsed)
{
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        currentOpticFlow = optic_flow;
        frameReady = true;
        t = timeElapsed;
    }

    frameCondition.notify_one();
    
}

void AngleNavigation::angleControl(const float angle) {
    
    target_angle = angle;
    alphabot->start();
    if (angle > 0) {
        alphabot->setLeftWheelSpeed(0.05f);
    } else if (angle < 0) {
        alphabot->setRightWheelSpeed(0.05f);
    }
    motorCondition = true;

}




void AngleNavigation::MovParamsCallback(std::function<void(MovParams)> callback)
{
    MovCallback = callback;
    
}

void AngleNavigation::processFilter()
{
    while (running)
    {
        static float timeElapsed = 0;
        cv::Vec2d optic_flow;
        {
            std::unique_lock<std::mutex> lock(frameMutex);
            frameCondition.wait(lock, [this] { return frameReady || !running; });
            if (!running)
                {
                    break;
                } 
                            
            frameReady = false;
            optic_flow = currentOpticFlow;
            timeElapsed = t;
        }

        
        //the differential move distance of the opticflow.
        float x = std::sqrt(optic_flow[0]*optic_flow[0] + optic_flow[1] *optic_flow[1]);
        float filtered_signal = low_pass.filter(x);


 
        if (motorCondition) {
            
            float s;// the arc length for the frame center movement in the real world.
            s = filtered_signal/p;//divide the proportion of opticflow value to the distance in the real world.
            static float angle;
            angle +=  s/r;
            
            cv::Mat transformMatrix = (cv::Mat_<float>(2, 3) << cos(angle), -sin(angle), lidar_r*0.5*cos(M_PI-angle),
                                                  sin(angle), cos(angle), lidar_r * 0.5*sin(M_PI-angle));

            //stop the motor when it arrived at target_angle
            
            current_angle = angle*180/M_PI;
            float angularVelocity = current_angle / timeElapsed;
            std::cout  <<"t: "<< timeElapsed <<", angle: "<< current_angle <<", V: "<< angularVelocity << ", optic_flow: " << optic_flow << ", s: " << s <<", signal: "<< x << ", Filtered_signal: " << filtered_signal << "," <<  std::endl;
            std::cout << "tramsformMatrix: " << transformMatrix << std::endl;
            MovParams params(transformMatrix, current_angle, angularVelocity, timeElapsed);
            
            if (abs(current_angle) >= abs(target_angle)) {
                
                motorCondition = false;
                alphabot->stop();
                std::cout << "stop at" << current_angle << std::endl;
                std::cout << "tramsformMatrix: " << transformMatrix << std::endl;
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Add a small delay to avoid busy waiting
                //s = 0;
                current_angle = 0;
                angle = 0;
            }
        } 

    }
}
/*
cv::Mat AngleNavigation::getMatrix(){ 
    //const MovParams params;
    return params.transformMatrix;;
}

b2Transform AngleNavigation::getb2Transform(){ 
    //const MovParams params;
    result.p.x = -(params.transformMatrix.at<double>(0, 2)) / params.t;
    result.p.y =-((params.transformMatrix.at<double>(1, 2)) / params.t;
    result.q.Set(acos(params.transformMatrix.at<double>(0, 0)) / params.t);
    return result;
}
*/





