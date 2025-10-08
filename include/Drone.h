#ifndef DRONE_H
#define DRONE_H

#include <array>
#include <cstdint>
#include <cmath>

#include "RemoteAPIClient.h"

class Drone
{
public:
    static constexpr std::uint64_t s_propellersCount = 4;

    explicit Drone(RemoteAPIObject::sim& sim) :
        m_sim{ &sim },
        m_drone{ sim.getObject("/Quadcopter/base/target") },
        m_respondables{
            sim.getObject("/Quadcopter/propeller[0]/respondable"),
            sim.getObject("/Quadcopter/propeller[1]/respondable"),
            sim.getObject("/Quadcopter/propeller[2]/respondable"),
            sim.getObject("/Quadcopter/propeller[3]/respondable")
        },
        m_visionSensor{ sim.getObject("/Quadcopter/visionSensor") },
        m_gyroSensorScript{ sim.getScript(sim.scripttype_childscript, "/Quadcopter/gyroSensor/Script") }
    {
        std::cout << m_drone << std::endl;
    }

    [[nodiscard]] std::tuple<std::vector<std::uint8_t>, std::vector<std::int64_t>> getImage() const
    {
        return m_sim->getVisionSensorImg(m_visionSensor);
    }

    [[nodiscard]] std::vector<double> getGyroData() const
    {
        json data = m_sim->callScriptFunction("getGyroData", m_gyroSensorScript)[0];

        if (!data.is_array())
        {
            return { 0.0, 0.0, 0.0 };
        }

        return {
            data[0].as<double>(),
            data[1].as<double>(),
            data[2].as<double>()
        };
    }

    [[nodiscard]] double getAltitude() const
    {
        return m_sim->getObjectPosition(m_drone)[2];
    }

    void setAngularVelocities(const std::array<double, s_propellersCount>& angularVelocities)
    {
        m_angularVelocities = angularVelocities;
    }

    void update() const
    {
        for (std::uint64_t i = 0; i < s_propellersCount; ++i)
        {
            std::vector<double> angles = m_sim->getObjectOrientation(m_respondables[i]);
            m_sim->addForceAndTorque(m_respondables[i], rotateForce(
                angles,
                m_angularVelocities[i]
            ));
        }
    }

private:
    static std::vector<double> rotateForce(const std::vector<double>& angles, double thrust)
    {
        double cx = std::cos(angles[0]);
        double sx = std::sin(angles[0]);
        double cy = std::cos(angles[1]);
        double sy = std::sin(angles[1]);
        double cz = std::cos(angles[2]);
        double sz = std::sin(angles[2]);

        double R[3][3];
        R[0][0] = cz*cy;             R[0][1] = cz*sy*sx - sz*cx;   R[0][2] = cz*sy*cx + sz*sx;
        R[1][0] = sz*cy;             R[1][1] = sz*sy*sx + cz*cx;   R[1][2] = sz*sy*cx - cz*sx;
        R[2][0] = -sy;               R[2][1] = cy*sx;              R[2][2] = cy*cx;

        return {
            R[0][2] * thrust,
            R[1][2] * thrust,
            R[2][2] * thrust
        };
    }

    RemoteAPIObject::sim* m_sim;
    std::int64_t m_drone;
    std::array<std::int64_t, s_propellersCount> m_respondables;
    std::int64_t m_visionSensor;
    std::int64_t m_gyroSensorScript;

    std::array<double, s_propellersCount> m_angularVelocities{};
};

#endif
