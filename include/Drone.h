#ifndef DRONE_H
#define DRONE_H

#include <array>
#include <cstdint>
#include <opencv2/opencv.hpp>

#include "RemoteAPIClient.h"

class Drone
{
public:
    constexpr static std::uint64_t s_propellersCount = 4;

    explicit Drone(RemoteAPIObject::sim& sim);

    [[nodiscard]] std::pair<int, int> getImageSize() const;

    [[nodiscard]] cv::Mat getGrayscaleImage() const;

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
    std::pair<int, int> m_cameraFrameSize;

    std::array<double, s_propellersCount> m_angularVelocities{};

    const std::array<std::int64_t, s_propellersCount> m_propellerDirections{ 1, -1, 1, -1 };
    const double kf = 3e-6;
    const double km = 3e-7;
};

#endif
