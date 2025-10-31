#ifndef VECMOVE_H
#define VECMOVE_H

#include "Drone.h"
#include "VecDown.h"
#include "CameraOpticalFlow.h"

class VecMove
{
public:
    explicit VecMove(const Drone& drone);

    void calc();

    [[nodiscard]] cv::Point2f getVecMove() const;

private:
    const Drone* m_drone;
    VecDown m_vecDown;
    CameraOpticalFlow m_cameraOpticalFlow;
    cv::Point2f m_vecMove;
    bool m_hasPrev = false;
};

#endif
