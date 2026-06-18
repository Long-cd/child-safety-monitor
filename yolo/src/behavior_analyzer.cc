#include "behavior_analyzer.h"
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int compute_center_point(pose_detect_result *det, float *cx, float *cy)
{
    int indices[4] = {5, 6, 11, 12};
    float sum_x = 0, sum_y = 0;
    int valid = 0;

    for (int i = 0; i < 4; i++) {
        int idx = indices[i];
        if (det->keypoints[idx][2] >= KP_CONF_THRESHOLD) {
            sum_x += det->keypoints[idx][0];
            sum_y += det->keypoints[idx][1];
            valid++;
        }
    }

    if (valid == 0)
        return -1;

    *cx = sum_x / valid;
    *cy = sum_y / valid;
    return 0;
}

static int compute_foot_center(pose_detect_result *det, float *fx, float *fy)
{
    float lx = det->keypoints[15][0], ly = det->keypoints[15][1];
    float rx = det->keypoints[16][0], ry = det->keypoints[16][1];
    float lc = det->keypoints[15][2], rc = det->keypoints[16][2];

    bool left_ok  = (lc >= KP_CONF_THRESHOLD);
    bool right_ok = (rc >= KP_CONF_THRESHOLD);

    if (!left_ok && !right_ok)
        return -1;

    if (left_ok && right_ok) {
        *fx = (lx + rx) / 2.0f;
        *fy = (ly + ry) / 2.0f;
    } else if (left_ok) {
        *fx = lx;
        *fy = ly;
    } else {
        *fx = rx;
        *fy = ry;
    }
    return 0;
}

static float compute_tilt_angle(float cx, float cy, float fx, float fy)
{
    float dx = cx - fx;
    float dy = cy - fy;

    if (fabsf(dy) < 1e-6f)
        return 90.0f;

    float theta_rad = atanf(fabsf(dx) / fabsf(dy));
    return theta_rad * 180.0f / M_PI;
}

// angle at vertex B formed by A-B-C, using cosine law
// returns degrees, or -1 if keypoints invalid
static float joint_angle(pose_detect_result *det,
                         int idx_a, int idx_b, int idx_c)
{
    float ax = det->keypoints[idx_a][0], ay = det->keypoints[idx_a][1];
    float bx = det->keypoints[idx_b][0], by = det->keypoints[idx_b][1];
    float cx = det->keypoints[idx_c][0], cy = det->keypoints[idx_c][1];
    float ac = det->keypoints[idx_a][2];
    float bc = det->keypoints[idx_b][2];
    float cc = det->keypoints[idx_c][2];

    if (ac < KP_CONF_THRESHOLD || bc < KP_CONF_THRESHOLD || cc < KP_CONF_THRESHOLD)
        return -1.0f;

    // vectors BA and BC
    float v1x = ax - bx, v1y = ay - by;
    float v2x = cx - bx, v2y = cy - by;
    float len1 = sqrtf(v1x * v1x + v1y * v1y);
    float len2 = sqrtf(v2x * v2x + v2y * v2y);

    if (len1 < 1e-6f || len2 < 1e-6f)
        return -1.0f;

    float dot = v1x * v2x + v1y * v2y;
    float cos_theta = dot / (len1 * len2);
    if (cos_theta > 1.0f)  cos_theta = 1.0f;
    if (cos_theta < -1.0f) cos_theta = -1.0f;

    return acosf(cos_theta) * 180.0f / M_PI;
}

int analyze_behavior(pose_detect_result *det, int image_height, BehaviorResult *result)
{
    result->status = STATUS_NORMAL;
    result->balance_theta = 0;
    result->foot_height = 0;
    result->safe_height = image_height * SAFE_HEIGHT_RATIO;
    result->center_x = result->center_y = 0;
    result->foot_center_x = result->foot_center_y = 0;
    result->elbow_angle = -1;
    result->shoulder_angle = -1;
    result->knee_angle = -1;
    result->hip_angle = -1;
    result->head_torso_angle = -1;

    // ---- tilt-based imbalance (unchanged) ----
    float cx, cy, fx, fy;
    float theta = 0;
    bool have_center = false, have_foot = false;

    if (compute_center_point(det, &cx, &cy) == 0) {
        result->center_x = cx;
        result->center_y = cy;
        have_center = true;
    }

    if (compute_foot_center(det, &fx, &fy) == 0) {
        result->foot_center_x = fx;
        result->foot_center_y = fy;
        result->foot_height = fy;
        have_foot = true;
    }

    if (have_center && have_foot) {
        theta = compute_tilt_angle(cx, cy, fx, fy);
        result->balance_theta = theta;
    }

    // ---- joint-angle based climbing ----
    // left side
    float left_elbow    = joint_angle(det, 5, 7, 9);   // shoulder-elbow-wrist
    float left_shoulder = joint_angle(det, 7, 5, 11);  // elbow-shoulder-hip
    float left_knee     = joint_angle(det, 11, 13, 15); // hip-knee-ankle
    float left_hip      = joint_angle(det, 5, 11, 13);  // shoulder-hip-knee

    // right side
    float right_elbow    = joint_angle(det, 6, 8, 10);  // shoulder-elbow-wrist
    float right_shoulder = joint_angle(det, 8, 6, 12);  // elbow-shoulder-hip
    float right_knee     = joint_angle(det, 12, 14, 16); // hip-knee-ankle
    float right_hip      = joint_angle(det, 6, 12, 14);  // shoulder-hip-knee

    // store most extreme angles for display
    if (left_elbow >= 0 || right_elbow >= 0) {
        float le = (left_elbow  >= 0) ? left_elbow  : right_elbow;
        float re = (right_elbow >= 0) ? right_elbow : left_elbow;
        result->elbow_angle = (le > re) ? le : re;
    }
    if (left_shoulder >= 0 || right_shoulder >= 0) {
        float ls = (left_shoulder >= 0) ? left_shoulder : right_shoulder;
        float rs = (right_shoulder >= 0) ? right_shoulder : left_shoulder;
        result->shoulder_angle = (ls < rs) ? ls : rs;
    }
    if (left_knee >= 0 || right_knee >= 0) {
        float lk = (left_knee >= 0) ? left_knee : right_knee;
        float rk = (right_knee >= 0) ? right_knee : left_knee;
        result->knee_angle = (lk > rk) ? lk : rk;
    }
    if (left_hip >= 0 || right_hip >= 0) {
        float lh = (left_hip >= 0) ? left_hip : right_hip;
        float rh = (right_hip >= 0) ? right_hip : left_hip;
        result->hip_angle = (lh > rh) ? lh : rh;
    }

    // ---- head/torso angle for leaning detection ----
    float lsx = det->keypoints[5][0], lsy = det->keypoints[5][1], lsc = det->keypoints[5][2];
    float rsx = det->keypoints[6][0], rsy = det->keypoints[6][1], rsc = det->keypoints[6][2];
    float lhx = det->keypoints[11][0], lhy = det->keypoints[11][1], lhc = det->keypoints[11][2];
    float rhx = det->keypoints[12][0], rhy = det->keypoints[12][1], rhc = det->keypoints[12][2];
    float nx  = det->keypoints[0][0],  ny  = det->keypoints[0][1],  nc  = det->keypoints[0][2];

    if (lsc >= KP_CONF_THRESHOLD && rsc >= KP_CONF_THRESHOLD &&
        lhc >= KP_CONF_THRESHOLD && rhc >= KP_CONF_THRESHOLD &&
        nc  >= KP_CONF_THRESHOLD) {
        float shoulder_mx = (lsx + rsx) / 2.0f, shoulder_my = (lsy + rsy) / 2.0f;
        float hip_mx      = (lhx + rhx) / 2.0f, hip_my      = (lhy + rhy) / 2.0f;

        // torso vector: hip_mid → shoulder_mid
        float tx = shoulder_mx - hip_mx, ty = shoulder_my - hip_my;
        // head vector: shoulder_mid → nose
        float hx = nx - shoulder_mx, hy = ny - shoulder_my;

        float t_len = sqrtf(tx * tx + ty * ty);
        float h_len = sqrtf(hx * hx + hy * hy);

        if (t_len > 1e-6f && h_len > 1e-6f) {
            float dot = tx * hx + ty * hy;
            float cos_theta = dot / (t_len * h_len);
            if (cos_theta > 1.0f)  cos_theta = 1.0f;
            if (cos_theta < -1.0f) cos_theta = -1.0f;
            result->head_torso_angle = acosf(cos_theta) * 180.0f / M_PI;
        }
    }

    // climbing: arm (elbow bent + arm raised) OR leg (knee bent + hip bent)
    // must be upright — if tilted beyond balance threshold, it's imbalance
    //
    // elbow: small angle = bent (straight arm ≈ 180°, bent ≈ 90°)
    // shoulder: large angle = arm raised (arm down ≈ 0°, raised ≈ 90°+)
    // knee: small angle = bent (straight ≈ 180°, bent ≈ 90°)
    // hip: small angle = bent (straight ≈ 180°, bent ≈ 90°)
    bool arm_climb = false;
    if (left_elbow >= 0 && left_shoulder >= 0)
        arm_climb |= (left_elbow < T_ELBOW_BEND && left_shoulder > T_SHOULDER_ANGLE);
    if (right_elbow >= 0 && right_shoulder >= 0)
        arm_climb |= (right_elbow < T_ELBOW_BEND && right_shoulder > T_SHOULDER_ANGLE);

    bool leg_climb = false;
    if (left_knee >= 0 && left_hip >= 0)
        leg_climb |= (left_knee < T_KNEE_ANGLE && left_hip < T_HIP_ANGLE);
    if (right_knee >= 0 && right_hip >= 0)
        leg_climb |= (right_knee < T_KNEE_ANGLE && right_hip < T_HIP_ANGLE);

    bool is_climbing = arm_climb || leg_climb;
    // suppress climbing when body is clearly tilted (imbalance)
    if (have_center && have_foot && theta > T_BALANCE)
        is_climbing = false;

    // imbalance: tilt over threshold and not climbing
    bool is_balance_lost = (theta > T_BALANCE) && !is_climbing;

    if (is_climbing)
        result->status = STATUS_CLIMBING;
    else if (is_balance_lost)
        result->status = STATUS_BALANCE_LOST;

    return 0;
}

const char* status_to_string(PoseStatus status)
{
    switch (status) {
        case STATUS_BALANCE_LOST: return "\033[31m失衡\033[0m";
        case STATUS_CLIMBING:     return "\033[33m攀爬\033[0m";
        case STATUS_LEANING:      return "\033[35m探头\033[0m";
        case STATUS_NORMAL:
        default:                  return "\033[32m正常\033[0m";
    }
}

const char* status_label(PoseStatus status)
{
    switch (status) {
        case STATUS_BALANCE_LOST: return "BALANCE LOST";
        case STATUS_CLIMBING:     return "CLIMBING";
        case STATUS_LEANING:      return "LEANING";
        case STATUS_NORMAL:
        default:                  return "NORMAL";
    }
}
