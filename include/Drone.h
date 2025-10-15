#ifndef DRONE_H
#define DRONE_H

#include <array>
#include <cstdint>

#include "RemoteAPIClient.h"

class Drone
{
public:
    static constexpr std::uint64_t s_propellersCount = 4;

    explicit Drone(RemoteAPIObject::sim& sim);

    [[nodiscard]] std::tuple<std::vector<std::uint8_t>, std::vector<std::int64_t>> getImage() const;

    [[nodiscard]] std::vector<double> getGyroData() const;

    [[nodiscard]] double getAltitude() const;

    [[nodiscard]] std::vector<double> getPos() const; // for debug

    void setAngularVelocities(const std::array<double, s_propellersCount>& angularVelocities);

    void update() const;

private:
    static std::vector<double> rotateForce(const std::vector<double>& angles, double thrust);

    RemoteAPIObject::sim* m_sim;
    std::int64_t m_drone;
    std::array<std::int64_t, s_propellersCount> m_respondables;
    std::int64_t m_visionSensor;
    std::int64_t m_gyroSensorScript;

    std::array<double, s_propellersCount> m_angularVelocities{};
};

#endif
