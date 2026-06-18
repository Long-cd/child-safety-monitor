#ifndef _DANGER_MONITOR_H_
#define _DANGER_MONITOR_H_

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "behavior_analyzer.h"

#define PROXIMITY_THRESHOLD_M 0.5f
#define LEAN_DISTANCE_THRESHOLD_M 0.2f
#define SEARCH_RADIUS_PX 60
#define WINDOW_CLS_ID 1

typedef struct {
    float X, Y, Z;
    bool valid;
} Point3D;

typedef struct {
    int person_idx;
    PoseStatus behavior;
    float balance_theta;
    float elbow_angle;
    float shoulder_angle;
    float knee_angle;
    float hip_angle;
    float head_torso_angle;
    float nose_to_window_dist;
    float min_dist_to_object;    // meters
    Point3D closest_obj_pt;
    Point3D closest_kp_pt;
    int closest_kp_id;
} PersonDangerInfo;

typedef struct {
    int frame_id;
    bool balance_alert;
    bool climbing_alert;
    bool leaning_alert;
    bool proximity_alert;
    float min_distance_m;
    float cpu_usage;
    float npu_usage;
    std::vector<PersonDangerInfo> persons;
    std::vector<std::string> object_names;
} DangerReport;

int analyze_danger(pose_detect_result_list* pose_results, int pose_ret,
                   void* seg_results, int seg_ret,
                   cv::Mat& depth_map, image_buffer_t* src_img,
                   float fx, float fy, float cx, float cy,
                   DangerReport* report);

bool should_save(const DangerReport& report);

void save_danger_frame(const DangerReport& report,
                       image_buffer_t* src_img,
                       cv::Mat& depth_map,
                       const std::string& out_dir);

std::string build_danger_json(const DangerReport& report);

#endif
