#include <iostream>
#include <windows.h>

#include "RemoteAPIClient.h"
#include "Drone.h"
#include "CameraOpticalFlow.h"

void showOpticalFlow(const cv::Mat& grayFrame,
                     const std::vector<cv::Point2f>& flowVec,
                     const std::pair<int, int>& frameSize,
                     const int step = 8,
                     const double resizeFactor = 1.5)
{
    cv::Mat display;
    cv::cvtColor(grayFrame, display, cv::COLOR_GRAY2BGR);
    cv::resize(display, display, cv::Size(), resizeFactor, resizeFactor, cv::INTER_LINEAR);

    for (int y = 0; y < frameSize.second; y += step)
    {
        for (int x = 0; x < frameSize.first; x += step)
        {
            const cv::Point2f& f = flowVec[y * frameSize.first + x];
            cv::Point2f p1(x * resizeFactor, y * resizeFactor);
            cv::Point2f p2 = p1 + f;

            if (cv::norm(f) > 1.0)
            {
                cv::arrowedLine(display, p1, p2, cv::Scalar(0, 255, 0), 1, cv::LINE_AA, 0, 0.3);
            }
        }
    }

    cv::imshow("Optical Flow", display);
    cv::waitKey(1);
}

int main(int argc, char* argv[])
{
    RemoteAPIClient client;
    RemoteAPIObject::sim sim = client.getObject().sim();

    Drone drone(sim);
    sim.setStepping(true);
    sim.startSimulation();

    auto [width, height] = drone.getImageSize();

    CameraOpticalFlow flow({ width, height });

    double t = 0.0;

    while (true)
    {
        t = sim.getSimulationTime();

        auto imgData = drone.getGrayscaleImage();
        if (!imgData.empty())
        {
            auto flowVec = flow.calcOpticalFlow(imgData);
            showOpticalFlow(imgData, flowVec, { width, height }, 16);
        }

        if (cv::waitKey(10) == 27)
        {
            break;
        }

        if (GetAsyncKeyState(VK_UP))
        {
            drone.setAngularVelocities({ 2000.0, 2000.0, 2000.0, 2000.0 });
        }
        else if (GetAsyncKeyState(VK_LEFT))
        {
            drone.setAngularVelocities({ 2000.0, 2000.0, 1800.0, 1800.0 });
        }
        else if (GetAsyncKeyState(VK_RIGHT))
        {
            drone.setAngularVelocities({ 1800.0, 1800.0, 2000.0, 2000.0 });
        }
        else if (GetAsyncKeyState(VK_DOWN))
        {
            drone.setAngularVelocities({ 2000.0, 0, 2000.0, 0 });
        }
        else
        {
            drone.setAngularVelocities({ 0.0, 0.0, 0.0, 0.0 });
        }

        drone.update();

        sim.step();
    }

    sim.stopSimulation();

    return 0;
}