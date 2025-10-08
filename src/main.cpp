#include <iostream>
#include <vector>
#include <cstdint>
#include <windows.h>
#include <opencv4/opencv2/opencv.hpp>
#include <execution>
#include <numeric>
#include <thread>

#include "RemoteAPIClient.h"

#include "Drone.h"

#include <cmath>

double getVelocity(
    const std::tuple<std::vector<uint8_t>, std::vector<int64_t>>& oldFrame,
    const std::tuple<std::vector<uint8_t>, std::vector<int64_t>>& currFrame,
    const std::vector<double>& gyroData,
    double dt,
    double altitude,             // meters
    double hfov_deg = 90.0,      // camera horizontal FOV in degrees
    double vfov_deg = 60.0       // camera vertical FOV in degrees
)
{
    using namespace cv;

    // --- Validate input ---
    const auto& [oldData, oldRes] = oldFrame;
    const auto& [currData, currRes] = currFrame;

    if (oldRes.size() < 2 || currRes.size() < 2 || dt <= 1e-6 || altitude <= 0)
        return 0.0;

    const int width  = static_cast<int>(oldRes[0]);
    const int height = static_cast<int>(oldRes[1]);

    if (oldData.size() < static_cast<size_t>(width * height) ||
        currData.size() < static_cast<size_t>(width * height))
        return 0.0;

    // --- Prepare Mats ---
    Mat imgOld(height, width, CV_8UC1, const_cast<uint8_t*>(oldData.data()));
    Mat imgCurr(height, width, CV_8UC1, const_cast<uint8_t*>(currData.data()));
    Mat oldGray = imgOld.clone();
    Mat currGray = imgCurr.clone();

    // --- Preprocess (light blur) ---
    cv::GaussianBlur(oldGray, oldGray, Size(5,5), 1.2);
    cv::GaussianBlur(currGray, currGray, Size(5,5), 1.2);

    // --- Optical flow ---
    setUseOptimized(true);
    setNumThreads(std::thread::hardware_concurrency());

    Mat flow;
    calcOpticalFlowFarneback(oldGray, currGray, flow,
                                 0.5, 3, 15, 3, 5, 1.1, OPTFLOW_FARNEBACK_GAUSSIAN);

    // --- Compute average flow (parallel) ---
    const Point2f* flowPtr = flow.ptr<Point2f>(0);
    const int total = width * height;

    Point2f sum = std::reduce(
        std::execution::par_unseq,
        flowPtr, flowPtr + total,
        Point2f(0.0f, 0.0f),
        [](const Point2f& a, const Point2f& b) {
            return Point2f(a.x + b.x, a.y + b.y);
        });

    double vx_pix = sum.x / total;
    double vy_pix = sum.y / total;

    // --- Compensate for gyro (approx rotation) ---
    if (gyroData.size() >= 3) {
        double yaw   = gyroData[0]; // rad/s
        double pitch = gyroData[1]; // rad/s
        // roll ignored for downward camera

        // pixels moved due to rotation
        double fx = width  / 2.0;
        double fy = height / 2.0;
        vx_pix -= fx * yaw * dt;
        vy_pix -= fy * pitch * dt;
    }

    // --- Convert pixel motion to ground motion (m/s) ---
    double hfov = hfov_deg * CV_PI / 180.0;
    double vfov = vfov_deg * CV_PI / 180.0;

    double angle_per_pixel_x = hfov / width;
    double angle_per_pixel_y = vfov / height;

    // displacement (meters)
    double dx = altitude * std::tan(vx_pix * angle_per_pixel_x);
    double dy = altitude * std::tan(vy_pix * angle_per_pixel_y);

    // velocity (m/s)
    double vx = dx / dt;
    double vy = dy / dt;

    double velocity = std::sqrt(vx * vx + vy * vy);

    if (!std::isfinite(velocity) || velocity > 200.0) // sanity limit
        velocity = 0.0;

    return velocity;
}

int main(int argc, char* argv[])
{
    RemoteAPIClient client;
    RemoteAPIObject::sim sim = client.getObject().sim();

    Drone drone(sim);
    sim.setStepping(true);

    sim.startSimulation();

    std::tuple<std::vector<uint8_t>, std::vector<int64_t>> oldFrame;
    double t = 0.0;
    while (t < 30.0)
    {
        double prevT = t;
        t = sim.getSimulationTime();
        std::cout << "Simulation time: " << t << '\n';
        auto newFrame = drone.getImage();
        if (std::get<0>(oldFrame).empty())
        {
            std::get<0>(oldFrame) = std::get<0>(newFrame);
        }
        std::cout << getVelocity(oldFrame, newFrame, drone.getGyroData(), t - prevT, drone.getAltitude()) << '\n';

        if (GetAsyncKeyState(VK_UP))
        {
            drone.setAngularVelocities({ 5.0, 5.0, 5.0, 5.0 });
        }
        else if (GetAsyncKeyState(VK_LEFT))
        {
            drone.setAngularVelocities({ 5.0, 5.0, 4.7, 4.7 });
        }
        else if (GetAsyncKeyState(VK_RIGHT))
        {
            drone.setAngularVelocities({ 4.7, 4.7, 5.0, 5.0 });
        }
        else
        {
            drone.setAngularVelocities({ 0.0, 0.0, 0.0, 0.0 });
        }

        drone.update();

        sim.step();

        oldFrame = newFrame;
    }

    sim.stopSimulation();

    return 0;
}