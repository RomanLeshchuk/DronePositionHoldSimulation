#include <iostream>
#include <vector>
#include <cstdint>
#include <windows.h>
#include <opencv4/opencv2/opencv.hpp>

#include "RemoteAPIClient.h"

#include "Drone.h"

double getVelocity(
    const std::tuple<std::vector<uint8_t>, std::vector<int64_t>>& oldFrame,
    const std::tuple<std::vector<uint8_t>, std::vector<int64_t>>& currFrame,
    const std::vector<double>& gyroData,
    double dt)
{
    // --- Extract image and resolution ---
    const auto& [oldData, oldRes] = oldFrame;
    const auto& [currData, currRes] = currFrame;

    if (oldRes.size() < 2 || currRes.size() < 2)
        return -1.0;

    int width  = static_cast<int>(oldRes[0]);
    int height = static_cast<int>(oldRes[1]);

    if (oldData.size() < width * height || currData.size() < width * height)
        return -2.0;

    if (!gyroData.empty() && gyroData.size() < 3)
        return -3.0;

    // --- Convert to cv::Mat (grayscale) ---
    cv::Mat imgOld(height, width, CV_8UC1, const_cast<uint8_t*>(oldData.data()));
    cv::Mat imgCurr(height, width, CV_8UC1, const_cast<uint8_t*>(currData.data()));

    // --- Preprocessing: reduce noise ---
    cv::GaussianBlur(imgOld, imgOld, cv::Size(5,5), 1.0);
    cv::GaussianBlur(imgCurr, imgCurr, cv::Size(5,5), 1.0);

    // --- Calculate dense optical flow ---
    cv::Mat flow;
    cv::calcOpticalFlowFarneback(imgOld, imgCurr, flow,
                                 0.5, 3, 15, 3, 5, 1.2, 0);

    // --- Compute average displacement ---
    cv::Scalar meanFlow = cv::mean(flow);
    double vx = meanFlow[0]; // horizontal displacement in pixels
    double vy = meanFlow[1]; // vertical displacement in pixels

    // --- Compensate for rotation if gyro data is provided ---
    if (gyroData.size() >= 3) {
        double yaw   = gyroData[0];
        double pitch = gyroData[1];
        double roll  = gyroData[2];

        double fx = width / 2.0;   // approximate focal length in pixels
        double fy = height / 2.0;

        vx -= -fx * yaw;     // simple rotational compensation (yaw)
        vy -= -fy * pitch;   // simple rotational compensation (pitch)
        // roll is ignored for simplicity
    }

    // --- Convert displacement to velocity (pixels/sec) ---
    vx /= dt;
    vy /= dt;

    // --- Return magnitude of velocity ---
    return std::sqrt(vx*vx + vy*vy);
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
        std::cout << getVelocity(oldFrame, newFrame, drone.getGyroData(), t - prevT) << '\n';

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