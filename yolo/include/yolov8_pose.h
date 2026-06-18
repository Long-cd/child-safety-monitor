#ifndef _RKNN_DEMO_YOLOV8_POSE_H_
#define _RKNN_DEMO_YOLOV8_POSE_H_

#include "rknn_api.h"
#include "common.h"

typedef struct {
    rknn_context rknn_ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr* input_attrs;
    rknn_tensor_attr* output_attrs;
    int model_channel;
    int model_width;
    int model_height;
    bool is_quant;
} rknn_pose_context_t;

#include "postprocess_pose.h"

int init_yolov8_pose_model(const char* model_path, rknn_pose_context_t* app_ctx);
int release_yolov8_pose_model(rknn_pose_context_t* app_ctx);
int inference_yolov8_pose_model(rknn_pose_context_t* app_ctx, image_buffer_t* img, pose_detect_result_list* od_results);

#endif
