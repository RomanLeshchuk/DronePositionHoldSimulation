#include <iostream>
#include <windows.h>

#include "RemoteAPIClient.h"

#include "Drone.h"

int main(int argc, char* argv[])
{
    RemoteAPIClient client;
    RemoteAPIObject::sim sim = client.getObject().sim();

    Drone drone(sim);
    sim.setStepping(true);

    sim.startSimulation();

    double t = 0.0;
    while (t < 100.0)
    {
        t = sim.getSimulationTime();

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
    }

    sim.stopSimulation();

    return 0;
}