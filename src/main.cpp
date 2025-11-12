#include <iostream>
#include <windows.h>

#include "RemoteAPIClient.h"

#include "Drone.h"
#include "VecMove.h"

void showOpticalFlow(const cv::Mat& grayFrame,
                     const VecMove& vecMove,
                     const std::pair<int, int>& frameSize,
                     const int step = 8,
                     const float resizeFactor = 1.5f)
{
    cv::Mat display;
    cv::cvtColor(grayFrame, display, cv::COLOR_GRAY2BGR);
    cv::resize(display, display, cv::Size(), resizeFactor, resizeFactor, cv::INTER_LINEAR);

    const cv::Point2f move = vecMove.getVecMove();
    std::cout << move.x << ' ' << move.y << std::endl;

    // === Axis parameters ===
    const int axisBoxSize = 100; // half-size of the coordinate box (in pixels)
    const cv::Point center(120, display.rows - 120); // center of mini-axis system
    const int thickness = 2;

    // Draw coordinate box outline (-1 to 1 range)
    cv::rectangle(display,
        center + cv::Point(-axisBoxSize, -axisBoxSize),
        center + cv::Point(axisBoxSize, axisBoxSize),
        cv::Scalar(80, 80, 80),
        1,
        cv::LINE_AA
    );

    // Draw X and Y axes (through center)
    cv::arrowedLine(display, center, center + cv::Point(axisBoxSize, 0),
                    cv::Scalar(0, 255, 0), thickness, cv::LINE_AA, 0, 0.3); // +X
    cv::arrowedLine(display, center, center - cv::Point(axisBoxSize, 0),
                    cv::Scalar(0, 255, 0), thickness, cv::LINE_AA, 0, 0.3); // -X

    cv::arrowedLine(display, center, center - cv::Point(0, axisBoxSize),
                    cv::Scalar(0, 255, 0), thickness, cv::LINE_AA, 0, 0.3); // +Y (up)
    cv::arrowedLine(display, center, center + cv::Point(0, axisBoxSize),
                    cv::Scalar(0, 255, 0), thickness, cv::LINE_AA, 0, 0.3); // -Y (down)

    // Labels
    cv::putText(display, "+X", center + cv::Point(axisBoxSize + 10, 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    cv::putText(display, "-X", center - cv::Point(axisBoxSize + 25, -15),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    cv::putText(display, "+Y", center - cv::Point(15, axisBoxSize + 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    cv::putText(display, "-Y", center + cv::Point(-15, axisBoxSize + 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

    // === Normalize and draw the movement vector ===
    // Clamp move.x and move.y to [-1, 1] to fit inside the frame
    float xNorm = std::clamp(move.x, -1.0f, 1.0f);
    float yNorm = std::clamp(move.y, -1.0f, 1.0f);

    // Convert normalized [-1,1] coords into screen pixels
    cv::Point vecEnd(
        static_cast<int>(center.x + xNorm * axisBoxSize),
        static_cast<int>(center.y - yNorm * axisBoxSize) // minus because screen Y is downward
    );

    cv::arrowedLine(display, center, vecEnd,
                    cv::Scalar(255, 0, 0), 2, cv::LINE_AA, 0, 0.3);

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

    VecMove vecMove(drone);

    while (true)
    {
        auto imgData = drone.getGrayscaleImage();
        if (!imgData.empty())
        {;
            vecMove.calc();
            showOpticalFlow(imgData, vecMove, { drone.cameraInfo.resolutionX, drone.cameraInfo.resolutionY }, 16, 1.5);
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
            drone.setAngularVelocities({ 2000.0, 2000.0, 1900.0, 1900.0 });
        }
        else if (GetAsyncKeyState(VK_RIGHT))
        {
            drone.setAngularVelocities({ 1900.0, 1900.0, 2000.0, 2000.0 });
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
