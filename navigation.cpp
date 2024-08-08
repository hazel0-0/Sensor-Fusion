#include "navigation.h"
#include "math.h"

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
        frameCondition.notify_all();
        if (filteringThread.joinable()) {
            filteringThread.join();
        }
        
       
    }
    alphabot->setRightWheelSpeed(0);
    alphabot->setLeftWheelSpeed(0);
    alphabot->stop();
    
}

void AngleNavigation::filter(const cv::Vec2d optic_flow)
{
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        currentOpticFlow = optic_flow;
        frameReady = true;
        
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

void AngleNavigation::processFilter()
{
    while (running)
    {
		static double signal = 0;
		static double filtered_signal = 0;
        cv::Vec2d optic_flow;
        {
            std::unique_lock<std::mutex> lock(frameMutex);
            frameCondition.wait(lock, [this] { return frameReady || !running; });
            if (!running)
                {filtered_signal = 0;break;} // Reset the filtered signal when stopping
                            
            frameReady = false;
            optic_flow = currentOpticFlow;
        }

        cv::Vec2d optic_flow_filtered = optic_flow;
        signal += optic_flow[0];

        optic_flow_filtered[0] = low_pass.filter(optic_flow[0]);
        //optic_flow_filtered[0] = band_stop.filter(optic_flow_filtered[0]);
        
        filtered_signal += optic_flow_filtered[0];
        std::cout << "signal: " << signal << std::endl;
        std::cout << "Filtered signal: " << filtered_signal << std::endl;
        
        current_angle = filtered_signal / 25;
        
        std::cout << "current angle: " << current_angle << std::endl;

        //stop the motor when it arrived at target_angle
 
        if (motorCondition) {
            if (abs(current_angle) >= abs(target_angle)) {
                alphabot->setRightWheelSpeed(0);
                alphabot->setLeftWheelSpeed(0);
                alphabot->stop();
                motorCondition = false;
                filtered_signal = 0;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Add a small delay to avoid busy waiting

    }
}



float AngleNavigation::getAngle()
{
    std::cout <<" angular:"  << current_angle << std::endl;
	return current_angle;
}



