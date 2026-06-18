#ifndef _RKNN_YOLOV8_POSE_DEMO_POSTPROCESS_H_
#define _RKNN_YOLOV8_POSE_DEMO_POSTPROCESS_H_

#include <stdint.h>
#include <vector>
#include "rknn_api.h"
#include "common.h"
#include "image_utils.h"

#ifndef OBJ_NUMB_MAX_SIZE
#define OBJ_NUMB_MAX_SIZE 128
#endif
#define POSE_OBJ_CLASS_NUM 1
#define POSE_NMS_THRESH 0.4
#define POSE_BOX_THRESH 0.5

typedef struct {
    image_rect_t box;
    float keypoints[17][3];
    float prop;
    int cls_id;
} pose_detect_result;

typedef struct {
    int id;
    int count;
    pose_detect_result results[OBJ_NUMB_MAX_SIZE];
} pose_detect_result_list;

int init_pose_post_process();
void deinit_pose_post_process();
char *pose_cls_to_name(int cls_id);
int pose_post_process(rknn_pose_context_t *app_ctx, void *outputs, letterbox_t *letter_box, float conf_threshold, float nms_threshold, pose_detect_result_list *od_results);

#endif
