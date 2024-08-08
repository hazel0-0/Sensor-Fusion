#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "opticFlow.h"
#include "Iir.h"
#include "alphabot.h"
#include "pigpio.h"
#include "iostream"

class AngleNavigation {
public:
    AngleNavigation();
    ~AngleNavigation();
    
    void start();
    void stop();
    void filter(const cv::Vec2d optic_flow);
    float getAngle();
    void angleControl(const float angle);

private:
    static const int order = 3; 
    const int FPS = 8;
    const double cutoff_frequency = 2.0;
    const double DC = 0.0;
    const double band_width = 0.5;


    void processFilter();
    
    float current_angle = 0;
    float target_angle = 0;
    bool running;
    bool frameReady;
    bool motorCondition;

    cv::Vec2d currentOpticFlow;
    std::mutex frameMutex;
    std::condition_variable frameCondition;
    std::thread filteringThread;
    
    Iir::Butterworth::LowPass<order> low_pass;
    Iir::Butterworth::BandStop<order> band_stop;
    const int reset_hz = 10;
    
    
    AlphaBot *alphabot; 
    OpticalFlowTracker opticflow;

};

#endif // NAVIGATION_H
