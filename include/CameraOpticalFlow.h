#ifndef CAMERAOPTICALFLOW_H
#define CAMERAOPTICALFLOW_H

#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>
#include <cstdint>

class CameraOpticalFlow
{
public:
    explicit CameraOpticalFlow(const std::pair<std::uint64_t, std::uint64_t>& frameSize);

    std::vector<cv::Point2f> calcOpticalFlow(const cv::Mat& grayFrame);

private:
    std::pair<std::uint64_t, std::uint64_t> m_frameSize;
    cv::Mat m_prevFrame;
    bool m_hasPrev = false;
};

#endif