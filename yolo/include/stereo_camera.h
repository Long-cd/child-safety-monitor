#ifndef _STEREO_CAMERA_H_
#define _STEREO_CAMERA_H_

#include <opencv2/opencv.hpp>
#if __has_include(<opencv2/ximgproc.hpp>)
#define HAVE_XIMGPROC
#include <opencv2/ximgproc.hpp>
#endif
#include <string>
#include "common.h"

class StereoCamera {
public:
    StereoCamera();
    ~StereoCamera();

    int init(const std::string& calibFile, int camId, const std::string& gstPipe,
             float scale = 0.5f, bool fastMode = false);
    bool grab();
    void release();

    cv::Mat getRectL()  const { return m_rectLcrop; }
    cv::Mat getDepth()  const { return m_depthShow; }
    int getWidth()      const { return m_roiFull.width; }
    int getHeight()     const { return m_roiFull.height; }
    float getFx() const { return m_fx; }
    float getFy() const { return m_fy; }
    float getCx() const { return m_cx_roi; }
    float getCy() const { return m_cy_roi; }

    void fillImageBuffer(image_buffer_t* buf);
    void pixelTo3D(int u, int v, float depth, float& X, float& Y, float& Z) const;

private:
    cv::VideoCapture m_cap;
    cv::Mat m_mtxL, m_distL, m_mtxR, m_distR, m_R, m_T;
    cv::Mat m_R1, m_R2, m_P1, m_P2, m_Q;
    cv::Mat m_mapLx, m_mapLy, m_mapRx, m_mapRy;
    cv::Mat m_Q_scaled;
    cv::Size m_imgSize, m_procSize;
    cv::Rect m_roi1, m_roi2, m_roiFull, m_roiProc;
    float m_scale;
    bool m_fastMode;
    int m_numDisp, m_numDispScaled;
    int m_halfW, m_h;

    cv::Ptr<cv::StereoSGBM> m_stereoL;
    cv::Ptr<cv::StereoMatcher> m_stereoR;
#ifdef HAVE_XIMGPROC
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> m_wls;
#endif

    cv::Mat m_rectL, m_rectR, m_rectLcrop;
    cv::Mat m_depthShow;

    float m_fx, m_fy, m_cx_roi, m_cy_roi;

    bool readYamlMat(const cv::FileStorage& fs, const std::string& key, cv::Mat& m);
};

#endif
