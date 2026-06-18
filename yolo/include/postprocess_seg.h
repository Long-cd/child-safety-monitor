#ifndef _RKNN_YOLOV8_SEG_DEMO_POSTPROCESS_H_
#define _RKNN_YOLOV8_SEG_DEMO_POSTPROCESS_H_

#include <stdint.h>
#include <vector>
#include "rknn_api.h"
#include "common.h"
#include "image_utils.h"

#ifndef OBJ_NUMB_MAX_SIZE
#define OBJ_NUMB_MAX_SIZE 128
#endif
#define SEG_OBJ_CLASS_NUM 3
#define SEG_NMS_THRESH 0.45
#define SEG_BOX_THRESH 0.25

#define PROTO_CHANNEL 32
#define PROTO_HEIGHT 160
#define PROTO_WEIGHT 160

#define N_CLASS_COLORS 20

typedef struct
{
    image_rect_t box;
    float prop;
    int cls_id;
} seg_detect_result;

typedef struct
{
    uint8_t *seg_mask;
} seg_segment_result;

typedef struct
{
    int id;
    int count;
    seg_detect_result results[OBJ_NUMB_MAX_SIZE];
    seg_segment_result results_seg[OBJ_NUMB_MAX_SIZE];
} seg_detect_result_list;

int init_seg_post_process();
void deinit_seg_post_process();
char *seg_cls_to_name(int cls_id);
int seg_post_process(rknn_seg_context_t *app_ctx, rknn_output *outputs, letterbox_t *letter_box, float conf_threshold, float nms_threshold, seg_detect_result_list *od_results);
int clamp(float val, int min, int max);

#endif
