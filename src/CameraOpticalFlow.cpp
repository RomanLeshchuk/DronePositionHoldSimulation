#include <opencv4/opencv2/opencv.hpp>

#include "CameraOpticalFlow.h"

CameraOpticalFlow::CameraOpticalFlow(const std::pair<std::uint64_t, std::uint64_t>& frameSize) :
    m_frameSize{ frameSize }
{
}

std::vector<cv::Point2f> CameraOpticalFlow::calcOpticalFlow(const cv::Mat& grayFrame)
{
    // Validate input frame type
    CV_Assert(grayFrame.type() == CV_8UC1);
    CV_Assert(grayFrame.cols == static_cast<int>(m_frameSize.first));
    CV_Assert(grayFrame.rows == static_cast<int>(m_frameSize.second));

    if (!m_hasPrev) {
        m_prevFrame = grayFrame.clone();
        m_hasPrev = true;
        return std::vector<cv::Point2f>(m_frameSize.first * m_frameSize.second, cv::Point2f(0.0f, 0.0f));
    }

    cv::Mat flow;
    cv::calcOpticalFlowFarneback(
        m_prevFrame, grayFrame, flow,
        0.5,   // pyramid scale
        3,     // levels
        15,    // window size
        3,     // iterations
        5,     // poly_n
        1.2,   // poly_sigma
        0      // flags
    );

    std::vector<cv::Point2f> flowVec;
    flowVec.reserve(m_frameSize.first * m_frameSize.second);

    for (int y = 0; y < flow.rows; ++y) {
        const cv::Point2f* rowPtr = flow.ptr<cv::Point2f>(y);
        flowVec.insert(flowVec.end(), rowPtr, rowPtr + flow.cols);
    }

    m_prevFrame = grayFrame.clone();

    return flowVec;
}