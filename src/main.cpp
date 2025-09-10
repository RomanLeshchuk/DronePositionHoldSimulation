#include <iostream>
#include <vector>
#include <string>
#include <windows.h>

#include "RemoteAPIClient.h"

int main(int argc, char* argv[])
{
    RemoteAPIClient client;
    RemoteAPIObject::sim sim = client.getObject().sim();

    int64_t visionSensorHandle = sim.getObject("/Quadcopter/visionSensor");

    std::vector<int64_t> propellerRespondables(4);
    std::vector<int64_t> propellerJoints(4); // currently we do not rotate joints, just apply force to them
    for (int i = 0; i < 4; ++i)
    {
        propellerRespondables[i] = sim.getObject("/Quadcopter/propeller[" + std::to_string(i) + "]/respondable");
        propellerJoints[i] = sim.getObject("/Quadcopter/propeller[" + std::to_string(i) + "]/joint");
    }

    sim.setStepping(true);

    sim.startSimulation();

    double t = 0.0;
    while (t < 30.0)
    {
        t = sim.getSimulationTime();
        std::cout << "Simulation time: " << t << '\n';
        auto [img, resolution] = sim.getVisionSensorImg(visionSensorHandle);
        bool hasBlack = false;
        for (int i = 0; i < resolution[0] * resolution[1]; i += 3)
        {
            if (img[i] == 0 && img[i + 1] == 0 && img[i + 2] == 0)
            {
                hasBlack = true;
                break;
            }
        }

        if (!hasBlack && GetAsyncKeyState('W') & 0x8000)
        {
            for (int64_t propellerRespondable : propellerRespondables)
            {
                sim.addForceAndTorque(propellerRespondable, std::vector<double>{ 0.0, 0.0, 3.0 });
            }
        }

        sim.step();
    }

    sim.stopSimulation();

    return 0;
}