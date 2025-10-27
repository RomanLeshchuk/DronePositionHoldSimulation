#include <iostream>
#include <windows.h>

#include "RemoteAPIClient.h"
#include "Drone.h"
#include "CameraOpticalFlow.h"
#include "VecDown.h"

void showOpticalFlow(const cv::Mat& grayFrame,
                     const CameraOpticalFlow& cameraOpticalFlow,
                     const VecDown& vecDown,
                     const std::pair<int, int>& frameSize,
                     const std::vector<double>& gyroData,
                     const int step = 8,
                     const float resizeFactor = 1.5f)
{
    cv::Mat display;
    cv::cvtColor(grayFrame, display, cv::COLOR_GRAY2BGR);
    cv::resize(display, display, cv::Size(), resizeFactor, resizeFactor, cv::INTER_LINEAR);

    for (int y = 0; y < frameSize.second; y += step)
    {
        for (int x = 0; x < frameSize.first; x += step)
        {
            const cv::Point2f& f = cameraOpticalFlow.getOpticalFlowAt(x, y);
            cv::Point2f p1(x * resizeFactor, y * resizeFactor);
            cv::Point2f p2 = p1 + f;

            if (cv::norm(f) > 1.0f)
            {
                cv::arrowedLine(display, p1, p2, cv::Scalar(0, 255, 0), 1, cv::LINE_AA, 0, 0.3);
            }
        }
    }

    cv::Point2f bottom = vecDown.getVecDown();
    cv::Point2f bottomDisplacement = vecDown.getVecDownDisplacement();

    cv::circle(display, { (int)(bottom.y * resizeFactor), (int)(bottom.x * resizeFactor) }, 5, cv::Scalar(0, 0, 255), cv::FILLED);
    cv::arrowedLine(
        display,
        { (int)(bottom.y * resizeFactor), (int)(bottom.x * resizeFactor) },
        { (int)((bottom.y + bottomDisplacement.y) * resizeFactor), (int)((bottom.x + bottomDisplacement.x) * resizeFactor) },
        cv::Scalar(255, 0, 0),
        2,
        cv::LINE_AA,
        0,
        0.3
    );

    cv::imshow("Bottom camera", display);
    cv::waitKey(1);
}

int main(int argc, char* argv[])
{
    RemoteAPIClient client;
    RemoteAPIObject::sim sim = client.getObject().sim();

    Drone drone(sim);
    sim.setStepping(true);
    sim.startSimulation();

    CameraOpticalFlow opticalFlow(drone);
    VecDown vecDown(drone);

    double t = 0.0;

    while (true)
    {
        t = sim.getSimulationTime();

        auto imgData = drone.getGrayscaleImage();
        if (!imgData.empty())
        {
            opticalFlow.calc();
            vecDown.calc();
            showOpticalFlow(imgData, opticalFlow, vecDown, { drone.cameraInfo.resolutionX, drone.cameraInfo.resolutionY }, drone.getGyroData(), 16, 1.5);
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
            drone.setAngularVelocities({ 2000.0, 2000.0, 1950.0, 1950.0 });
        }
        else if (GetAsyncKeyState(VK_RIGHT))
        {
            drone.setAngularVelocities({ 1950.0, 1950.0, 2000.0, 2000.0 });
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