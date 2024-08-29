#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "opticFlow.h"
#include "Iir.h"
#include "alphabot.h"
#include "pigpio.h"
#include "iostream"
#include <box2d/box2d.h>
struct MovParams {
    cv::Mat &transformMatrix;
    float &angle;
    float &anglular_speed; 
    float t;

    MovParams(cv::Mat transformMatrix_init, 
              float angle_init,
              float anglular_speed_init,
              float t_init) 
        : transformMatrix(transformMatrix_init), angle(angle_init), anglular_speed(anglular_speed_init), t(t_init) {}
};

class AngleNavigation {
public:
    AngleNavigation();
    ~AngleNavigation();
    void MovParamsCallback(std::function<void(MovParams)> callback);
    void start();
    void stop();
    void filter(const cv::Vec2d& optic_flow, const float timeElapsed);
    void angleControl(const float angle);
    cv::Mat getMatrix();
    b2Transform getb2Transform();

private:
    static const int order = 3; 
    const int FPS = 9;
    const double cutoff_frequency = 2;
    const double DC = 0.0;
    const double band_width = 0.5;
    
    // for the following parameters, the unit is centimeters.
    const float r =  15;//The radius from the frame center to the wheel(the center of the rotation).
    const float lidar_r = 6;//The radius from lidar to the wheel.
    const float p = 90.5;//the proportion of opticflow value to the distance in the real world. 
    /*
     when the unit is meter:
    const float r =  0.15;//The radius from the frame center to the wheel(the center of the rotation).
    const float lidar_r = 0.06;//The radius from lidar to the wheel.
    const float p = 9050;//the proportion of opticflow value to the distance in the real world.
     */
    void processFilter();
    
    float current_angle = 0;
    float target_angle = 0;
    float t;
    bool running;
    bool frameReady;
    bool motorCondition;
    b2Transform result;

    cv::Vec2d currentOpticFlow;
    std::mutex frameMutex;
    std::condition_variable frameCondition;
    std::thread filteringThread;
    
    Iir::Butterworth::LowPass<order> low_pass;
    Iir::Butterworth::BandStop<order> band_stop;
    const int reset_hz = 10;
    std::function<void(MovParams)> MovCallback;
    
    AlphaBot *alphabot; 
    OpticalFlowTracker opticflow;

};

#endif // NAVIGATION_H
