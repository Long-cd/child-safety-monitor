#ifndef _BEHAVIOR_ANALYZER_H_
#define _BEHAVIOR_ANALYZER_H_

#include "yolov8_pose.h"

#define KP_CONF_THRESHOLD 0.5f
#define T_BALANCE      10.0f
#define SAFE_HEIGHT_RATIO 0.4f

// climbing thresholds — joint angles in degrees (cosine law)
// elbow/knee/hip: angle must be BELOW threshold (bent = small angle)
// shoulder:       angle must be ABOVE threshold (arm raised = large angle)
#define T_ELBOW_BEND      130.0f   // 肘弯曲 > 50° → 180-50=130
#define T_SHOULDER_ANGLE  60.0f    // 手臂抬起 > 60°
#define T_KNEE_ANGLE      110.0f   // 膝弯曲 > 70° → 180-70=110
#define T_HIP_ANGLE       135.0f   // 髋弯曲 > 45° → 180-45=135
#define T_LEAN_ANGLE      40.0f

enum PoseStatus {
    STATUS_NORMAL = 0,
    STATUS_BALANCE_LOST = 1,
    STATUS_CLIMBING = 2,
    STATUS_LEANING = 3
};

typedef struct {
    PoseStatus status;
    float balance_theta;
    float foot_height;
    float safe_height;
    float center_x, center_y;
    float foot_center_x, foot_center_y;
    float elbow_angle;
    float shoulder_angle;
    float knee_angle;
    float hip_angle;
    float head_torso_angle;
} BehaviorResult;

int analyze_behavior(pose_detect_result *det, int image_height, BehaviorResult *result);
const char* status_to_string(PoseStatus status);
const char* status_label(PoseStatus status);

#endif
