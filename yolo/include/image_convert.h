#ifndef _IMAGE_CONVERT_H_
#define _IMAGE_CONVERT_H_

#include <opencv2/opencv.hpp>
#include "common.h"

// Convert image_buffer_t (RGB) to cv::Mat (BGR)
inline cv::Mat rgb_buffer_to_bgr_mat(image_buffer_t* src)
{
    cv::Mat bgr(src->height, src->width, CV_8UC3);
    for (int y = 0; y < src->height; y++) {
        uint8_t* d = bgr.ptr<uint8_t>(y);
        unsigned char* s = src->virt_addr + y * src->width * 3;
        for (int x = 0; x < src->width; x++) {
            d[x*3+0] = s[x*3+2];
            d[x*3+1] = s[x*3+1];
            d[x*3+2] = s[x*3+0];
        }
    }
    return bgr;
}

// Fill image_buffer_t (RGB) from cv::Mat (BGR)
inline void bgr_mat_to_rgb_buffer(cv::Mat& bgr, image_buffer_t* buf)
{
    memset(buf, 0, sizeof(*buf));
    buf->width  = bgr.cols;
    buf->height = bgr.rows;
    buf->format = IMAGE_FORMAT_RGB888;
    buf->size   = buf->width * buf->height * 3;
    buf->virt_addr = (unsigned char*)malloc(buf->size);

    for (int y = 0; y < buf->height; y++) {
        const uint8_t* src = bgr.ptr<uint8_t>(y);
        unsigned char* dst = buf->virt_addr + y * buf->width * 3;
        for (int x = 0; x < buf->width; x++) {
            dst[x*3+0] = src[x*3+2];
            dst[x*3+1] = src[x*3+1];
            dst[x*3+2] = src[x*3+0];
        }
    }
}

#endif
