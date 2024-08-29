#include "affineTrans.h"
#include <queue>
static float current_angle = 0.0f;
AffineTrans::AffineTrans() : ready(false), done(false), running(false) {}

AffineTrans::~AffineTrans() {
    stop();
}

void AffineTrans::start() {
    if (!running) {
        running = true;
        workerThread = std::thread(&AffineTrans::processTasks, this);
    }
}

void AffineTrans::stop() {
    if (running) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            running = false;
            cv.notify_all(); // Notify thread to exit
            current_angle = 0.0f;
        }
        if (workerThread.joinable()) {
            workerThread.join();
        }
    }
}

void AffineTrans::addTask(const cv::Vec2d opticFlow, std::vector<Point2f> current, std::vector<Point2f> previous, float timeElapsed) {
    {
        std::lock_guard<std::mutex> lock(mtx);
        taskQueue.push({opticFlow, current, previous, timeElapsed});

    }
    cv.notify_one(); // Notify the worker thread of new task
}

void AffineTrans::processTasks() {
    while (running) {
        Task task;

        {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [this] { return !taskQueue.empty() || !running; });

            if (!running && taskQueue.empty()) {
                break; // Exit the thread if stop was called and no more tasks
            }

            task = taskQueue.front();
            taskQueue.pop();
        }

        // Perform affine transform estimation
        affineTransEstimate(task.opticFlow, task.current, task.previous, task.timeElapsed);
    }
}

void AffineTrans::affineTransEstimate(const cv::Vec2d opticFlow, std::vector<Point2f> current, std::vector<Point2f> previous, float timeElapsed) {
    std::lock_guard<std::mutex> lock(mtx);
    cv::Vec2d displacement;
    std::vector <Point2f> previousTmp = previous;

    if (previousTmp.empty() || current.empty() || previousTmp == current) {
        done = true;
        cv.notify_all();
        return;
    }

    int diff = current.size() - previousTmp.size();
    if (diff > 0) {
        for (int i = 0; i < abs(diff); i++) {
            previousTmp.push_back(previousTmp[0]);
        }
    } else if (diff < 0) {
        for (int i = 0; i < abs(diff); i++) {
            current.push_back(current[0]);
        }
    }

    cv::Mat transformMatrix = cv::estimateAffinePartial2D(previousTmp, current, cv::noArray(), cv::LMEDS);
    if (!transformMatrix.empty()) {

        displacement[0] = -(transformMatrix.at<double>(0, 2));
        displacement[1] = -(transformMatrix.at<double>(1, 2));

        result.p.x = displacement[0] / timeElapsed;
        result.p.y = displacement[1] / timeElapsed;
        

        result.q.Set(acos(transformMatrix.at<double>(0, 0)) / timeElapsed);
        
        float angle = acos(transformMatrix.at<double>(0, 0));
        float angularVelocity = angle / timeElapsed;

        current_angle += angle;
        

        float posAngle = 0;
        float tan = atan(result.p.y / result.p.x);
        if (result.p.y != 0 && result.p.x != 0) {
            posAngle = tan;
        }
        std::cout << "current_angle: " << current_angle*(180/3.1415) <<",  timeElapsed: "<< timeElapsed <<",  angle: "<< angle<<",  angularVelocity: "<< angularVelocity <<",  displacement: ["<< displacement[0] << ", " << displacement[1] <<"],  opticFlow: ["<< opticFlow[0]<< ", " << opticFlow[1] <<"],  "<< posAngle <<"transformMatrix:  ["<<  (transformMatrix.at<double>(0, 0)) << ","<< (transformMatrix.at<double>(0, 1))<<" ],  ["<< (transformMatrix.at<double>(1, 0))<<","<<(transformMatrix.at<double>(1, 1))<<"]"<< std::endl;

    }

    done = true;
    cv.notify_all();
}

b2Transform AffineTrans::getResult() {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [this]{ return done; });
    return result;
}


