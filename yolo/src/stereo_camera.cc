#include "stereo_camera.h"
#include "image_convert.h"
#include <iostream>
#include <cstring>

StereoCamera::StereoCamera() : m_scale(0.5f), m_fastMode(false),
    m_numDisp(128), m_numDispScaled(64), m_halfW(0), m_h(0) {}

StereoCamera::~StereoCamera() { release(); }

bool StereoCamera::readYamlMat(const cv::FileStorage& fs, const std::string& key, cv::Mat& m) {
    fs[key] >> m;
    return !m.empty();
}

int StereoCamera::init(const std::string& calibFile, int camId, const std::string& gstPipe,
                       float scale, bool fastMode)
{
    m_scale    = std::max(0.25f, std::min(1.0f, scale));
    m_fastMode = fastMode;

    // ---- load calibration ----
    cv::FileStorage fs(calibFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[Stereo] cannot open: " << calibFile << std::endl;
        return -1;
    }
    readYamlMat(fs, "mtxL",  m_mtxL);
    readYamlMat(fs, "distL", m_distL);
    readYamlMat(fs, "mtxR",  m_mtxR);
    readYamlMat(fs, "distR", m_distR);
    readYamlMat(fs, "R",     m_R);
    readYamlMat(fs, "T",     m_T);
    fs.release();
    std::cout << "[Stereo] calibration loaded" << std::endl;

    // ---- open camera ----
    if (!gstPipe.empty()) {
        m_cap.open(gstPipe, cv::CAP_GSTREAMER);
        std::cout << "[Stereo] GStreamer: " << gstPipe << std::endl;
    } else {
        m_cap.open(camId, cv::CAP_V4L2);
        if (!m_cap.isOpened())
            m_cap.open(camId);
        std::cout << "[Stereo] camera ID: " << camId << std::endl;
    }
    m_cap.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
    m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    cv::Mat frame;
    m_cap >> frame;
    if (frame.empty()) {
        std::cerr << "[Stereo] cannot read camera" << std::endl;
        return -1;
    }
    m_h     = frame.rows;
    m_halfW = frame.cols / 2;
    m_imgSize = cv::Size(m_halfW, m_h);
    std::cout << "[Stereo] resolution: " << frame.cols << "x" << frame.rows
              << " (single: " << m_halfW << "x" << m_h << ")" << std::endl;

    // ---- stereo rectification ----
    cv::stereoRectify(m_mtxL, m_distL, m_mtxR, m_distR, m_imgSize,
                      m_R, m_T, m_R1, m_R2, m_P1, m_P2, m_Q,
                      cv::CALIB_ZERO_DISPARITY, 0, m_imgSize, &m_roi1, &m_roi2);

    const int BLOCK_SIZE = 7;
    int x  = std::max({m_roi1.x, m_roi2.x, m_numDisp});
    int y  = std::max(m_roi1.y, m_roi2.y);
    int x2 = std::min(m_roi1.x + m_roi1.width,  m_roi2.x + m_roi2.width);
    int y2 = std::min(m_roi1.y + m_roi1.height, m_roi2.y + m_roi2.height);
    m_roiFull = cv::Rect(x, y, x2 - x, y2 - y);

    m_procSize = cv::Size(int(m_imgSize.width * m_scale), int(m_imgSize.height * m_scale));
    m_numDispScaled = int(m_numDisp * m_scale);
    m_numDispScaled = (m_numDispScaled / 16) * 16;
    m_numDispScaled = std::max(16, m_numDispScaled);

    int sx  = int(x  * m_scale), sy  = int(y  * m_scale);
    int sx2 = int(x2 * m_scale), sy2 = int(y2 * m_scale);
    m_roiProc = cv::Rect(sx, sy, sx2 - sx, sy2 - sy);

    std::cout << "[Stereo] proc size: " << m_procSize.width << "x" << m_procSize.height
              << "  numDisparities: " << m_numDispScaled
              << (m_fastMode ? "  [fast]" : "") << std::endl;

    cv::initUndistortRectifyMap(m_mtxL, m_distL, m_R1, m_P1, m_imgSize, CV_16SC2, m_mapLx, m_mapLy);
    cv::initUndistortRectifyMap(m_mtxR, m_distR, m_R2, m_P2, m_imgSize, CV_16SC2, m_mapRx, m_mapRy);

    // scale Q matrix
    m_Q_scaled = m_Q.clone();
    m_Q_scaled.at<double>(0, 0) /= 1.0 / m_scale;
    m_Q_scaled.at<double>(1, 1) /= 1.0 / m_scale;
    m_Q_scaled.at<double>(0, 3) *= m_scale;
    m_Q_scaled.at<double>(1, 3) *= m_scale;
    m_Q_scaled.at<double>(2, 3) *= m_scale;

    // intrinsics for ROI image (for 3D reconstruction)
    m_fx = (float)m_P1.at<double>(0, 0);
    m_fy = (float)m_P1.at<double>(1, 1);
    m_cx_roi = (float)(m_P1.at<double>(0, 2) - m_roiFull.x);
    m_cy_roi = (float)(m_P1.at<double>(1, 2) - m_roiFull.y);

    // ---- SGBM + optional WLS ----
    m_stereoL = cv::StereoSGBM::create(
        0, m_numDispScaled, BLOCK_SIZE,
        8 * 3 * BLOCK_SIZE * BLOCK_SIZE,
        32 * 3 * BLOCK_SIZE * BLOCK_SIZE,
        1, 0, 5, 100, 32);
    m_stereoL->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

    if (!m_fastMode) {
#ifdef HAVE_XIMGPROC
        m_stereoR = cv::ximgproc::createRightMatcher(m_stereoL);
        m_wls     = cv::ximgproc::createDisparityWLSFilter(m_stereoL);
        m_wls->setLambda(4000.0);
        m_wls->setSigmaColor(1.5);
#else
        std::cout << "[Stereo] ximgproc not available, forcing fast mode" << std::endl;
        m_fastMode = true;
#endif
    }

    return 0;
}

bool StereoCamera::grab()
{
    cv::Mat frame;
    m_cap >> frame;
    if (frame.empty()) return false;

    cv::Mat imgL = frame(cv::Rect(0, 0, m_halfW, m_h));
    cv::Mat imgR = frame(cv::Rect(m_halfW, 0, m_halfW, m_h));

    // rectify
    cv::remap(imgL, m_rectL, m_mapLx, m_mapLy, cv::INTER_LINEAR);
    cv::remap(imgR, m_rectR, m_mapRx, m_mapRy, cv::INTER_LINEAR);

    // scale for SGBM
    cv::Mat procL, procR;
    if (m_scale < 1.0f) {
        cv::resize(m_rectL, procL, m_procSize, 0, 0, cv::INTER_LINEAR);
        cv::resize(m_rectR, procR, m_procSize, 0, 0, cv::INTER_LINEAR);
    } else {
        procL = m_rectL;
        procR = m_rectR;
    }

    cv::Mat grayL, grayR;
    cv::cvtColor(procL, grayL, cv::COLOR_BGR2GRAY);
    cv::cvtColor(procR, grayR, cv::COLOR_BGR2GRAY);

    // disparity
    cv::Mat dispL, dispF32;
    m_stereoL->compute(grayL, grayR, dispL);

#ifdef HAVE_XIMGPROC
    if (!m_fastMode) {
        cv::Mat dispR, dispWLS;
        m_stereoR->compute(grayR, grayL, dispR);
        m_wls->filter(dispL, grayL, dispWLS, dispR);
        dispWLS.convertTo(dispF32, CV_32F, 1.0 / 16.0);
    } else
#endif
    {
        dispL.convertTo(dispF32, CV_32F, 1.0 / 16.0);
    }

    cv::Mat invalid = dispF32 < 0.5f;
    dispF32.setTo(1.0f, invalid);

    // reproject to 3D
    cv::Mat depth3D, depthFull;
    std::vector<cv::Mat> channels(3);
    cv::reprojectImageTo3D(dispF32, depth3D, m_Q_scaled, true);
    cv::split(depth3D, channels);
    depthFull = channels[2];

    cv::Mat depth = depthFull(m_roiProc).clone();
    cv::Mat invalidCrop = invalid(m_roiProc);
    depth.setTo(0.0f, invalidCrop);

    // post-process
    cv::Mat depthF32, tmp;
    depth.convertTo(tmp, CV_32F);
    cv::medianBlur(tmp, depthF32, 3);
    cv::bilateralFilter(depthF32, tmp, 5, 0.3, 15);
    depthF32 = tmp;
    depthF32.setTo(0.0f, invalidCrop);

    // resize depth to full ROI size
    if (m_scale < 1.0f) {
        cv::resize(depthF32, m_depthShow,
                   cv::Size(m_roiFull.width, m_roiFull.height),
                   0, 0, cv::INTER_LINEAR);
    } else {
        m_depthShow = depthF32;
    }

    // crop rectL to ROI (aligned with depth)
    m_rectLcrop = m_rectL(m_roiFull).clone();

    return true;
}

void StereoCamera::fillImageBuffer(image_buffer_t* buf)
{
    bgr_mat_to_rgb_buffer(m_rectLcrop, buf);
}

void StereoCamera::pixelTo3D(int u, int v, float depth, float& X, float& Y, float& Z) const
{
    Z = depth;
    X = (u - m_cx_roi) * Z / m_fx;
    Y = (v - m_cy_roi) * Z / m_fy;
}

void StereoCamera::release()
{
    if (m_cap.isOpened())
        m_cap.release();
    m_stereoL.reset();
    m_stereoR.reset();
#ifdef HAVE_XIMGPROC
    m_wls.reset();
#endif
}
