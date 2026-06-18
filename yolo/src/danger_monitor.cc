#include "yolov8_seg.h"
#include "danger_monitor.h"
#include "image_utils.h"
#include <cstdio>
#include <cmath>
#include <sys/stat.h>

static void ensureDir(const std::string& path) {
    mkdir(path.c_str(), 0755);
}

static float depthAt(cv::Mat& depth, float x, float y)
{
    int u = (int)(x + 0.5f), v = (int)(y + 0.5f);
    if (u < 0 || u >= depth.cols || v < 0 || v >= depth.rows) return -1.f;
    float d = depth.at<float>(v, u);
    return (d > 0.1f) ? d : -1.f;
}

static void pixelTo3D(float u, float v, float Z,
                       float fx, float fy, float cx, float cy,
                       Point3D& pt)
{
    if (Z <= 0.1f) { pt.valid = false; return; }
    pt.X = (u - cx) * Z / fx;
    pt.Y = (v - cy) * Z / fy;
    pt.Z = Z;
    pt.valid = true;
}

static float dist3D(const Point3D& a, const Point3D& b)
{
    if (!a.valid || !b.valid) return 1e9f;
    float dx = a.X - b.X, dy = a.Y - b.Y, dz = a.Z - b.Z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

// sample keypoints for proximity check: nose(0), L-wrist(9), R-wrist(10), L-ankle(15), R-ankle(16)
static const int PROXIMITY_KP[] = {0, 9, 10, 15, 16};
static const int PROXIMITY_KP_NUM = 5;

int analyze_danger(pose_detect_result_list* pose_results, int pose_ret,
                   void* seg_results_v, int seg_ret,
                   cv::Mat& depth_map, image_buffer_t* src_img,
                   float fx, float fy, float cx, float cy,
                   DangerReport* report)
{
    seg_detect_result_list* seg_results = (seg_detect_result_list*)seg_results_v;

    report->balance_alert   = false;
    report->climbing_alert  = false;
    report->leaning_alert   = false;
    report->proximity_alert = false;
    report->min_distance_m  = 1e9f;
    report->persons.clear();
    report->object_names.clear();

    if (depth_map.empty() || pose_ret != 0) return -1;

    int img_h = src_img->height;

    // ---- collect object mask (non-person, non-zero pixels) ----
    cv::Mat objMask;
    bool have_seg = (seg_ret == 0 && seg_results->count >= 1);
    if (have_seg) {
        uint8_t* mask = seg_results->results_seg[0].seg_mask;
        if (mask) {
            objMask = cv::Mat(img_h, src_img->width, CV_8UC1);
            for (int y = 0; y < img_h; y++) {
                for (int x = 0; x < src_img->width; x++) {
                    uint8_t v = mask[y * src_img->width + x];
                    objMask.at<uint8_t>(y, x) = (v > 0 && v != 1) ? 255 : 0; // !=1 means not person
                }
            }
        }
    }

    // ---- distance transform to find nearest object pixel ----
    cv::Mat dist2D;
    if (!objMask.empty() && cv::countNonZero(objMask) > 0) {
        cv::Mat invMask;
        cv::bitwise_not(objMask, invMask);
        cv::distanceTransform(invMask, dist2D, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    }

    // ---- collect object class names ----
    if (seg_ret == 0) {
        for (int i = 0; i < seg_results->count; i++) {
            const char* name = seg_cls_to_name(seg_results->results[i].cls_id);
            if (seg_results->results[i].cls_id != 0) // exclude person
                report->object_names.push_back(std::string(name));
        }
    }

    // ---- analyze each person ----
    if (pose_ret != 0 || pose_results->count == 0) {
        if (have_seg && !objMask.empty()) objMask.release();
        return 0;
    }

    for (int p = 0; p < pose_results->count; p++) {
        pose_detect_result* det = &pose_results->results[p];
        PersonDangerInfo pi;
        pi.person_idx = p;
        pi.min_dist_to_object = 1e9f;
        pi.closest_kp_id = -1;

        BehaviorResult bh;
        analyze_behavior(det, img_h, &bh);
        pi.behavior = bh.status;
        pi.balance_theta = bh.balance_theta;
        pi.elbow_angle = bh.elbow_angle;
        pi.shoulder_angle = bh.shoulder_angle;
        pi.knee_angle = bh.knee_angle;
        pi.hip_angle = bh.hip_angle;
        pi.head_torso_angle = bh.head_torso_angle;
        pi.nose_to_window_dist = -1.0f;

        if (bh.status == STATUS_BALANCE_LOST) report->balance_alert = true;
        if (bh.status == STATUS_CLIMBING)    report->climbing_alert = true;

        // ---- window leaning check ----
        if (have_seg && !objMask.empty() &&
            bh.head_torso_angle > T_LEAN_ANGLE &&
            det->keypoints[0][2] >= KP_CONF_THRESHOLD) {
            // build window-only mask
            cv::Mat windowMask(img_h, src_img->width, CV_8UC1, cv::Scalar(0));
            uint8_t* mask = seg_results->results_seg[0].seg_mask;
            if (mask) {
                for (int y = 0; y < img_h; y++) {
                    for (int x = 0; x < src_img->width; x++) {
                        uint8_t v = mask[y * src_img->width + x];
                        if (v == (WINDOW_CLS_ID + 1))
                            windowMask.at<uint8_t>(y, x) = 255;
                    }
                }
            }
            if (cv::countNonZero(windowMask) > 0) {
                float nx = det->keypoints[0][0], ny = det->keypoints[0][1];
                // find nearest window pixel to nose
                cv::Mat winDist;
                cv::Mat winInv;
                cv::bitwise_not(windowMask, winInv);
                cv::distanceTransform(winInv, winDist, cv::DIST_L2, cv::DIST_MASK_PRECISE);
                float d2d = winDist.at<float>((int)ny, (int)nx);

                int rx = (int)(nx + 0.5f), ry = (int)(ny + 0.5f);
                int best_wx = -1, best_wy = -1;
                float best_d2 = 1e9f;
                int r = (int)(d2d + 6.0f);
                for (int dy = -r; dy <= r; dy++) {
                    int wy = ry + dy;
                    if (wy < 0 || wy >= img_h) continue;
                    for (int dx = -r; dx <= r; dx++) {
                        int wx = rx + dx;
                        if (wx < 0 || wx >= src_img->width) continue;
                        if (windowMask.at<uint8_t>(wy, wx) == 0) continue;
                        float sd = sqrtf((float)(dx*dx + dy*dy));
                        if (sd < best_d2) { best_d2 = sd; best_wx = wx; best_wy = wy; }
                    }
                }

                if (best_wx >= 0) {
                    float Z_nose   = depthAt(depth_map, nx, ny);
                    float Z_window = depthAt(depth_map, (float)best_wx, (float)best_wy);
                    if (Z_nose > 0 && Z_window > 0) {
                        Point3D nose3d, win3d;
                        pixelTo3D(nx, ny, Z_nose, fx, fy, cx, cy, nose3d);
                        pixelTo3D((float)best_wx, (float)best_wy, Z_window, fx, fy, cx, cy, win3d);
                        float d3d = dist3D(nose3d, win3d);
                        pi.nose_to_window_dist = d3d;
                        if (d3d < LEAN_DISTANCE_THRESHOLD_M) {
                            pi.behavior = STATUS_LEANING;
                            report->leaning_alert = true;
                        }
                    }
                }
            }
            if (!windowMask.empty()) windowMask.release();
        }

        // ---- proximity check ----
        if (!dist2D.empty()) {
            for (int k = 0; k < PROXIMITY_KP_NUM; k++) {
                int kp_id = PROXIMITY_KP[k];
                float kx = det->keypoints[kp_id][0];
                float ky = det->keypoints[kp_id][1];
                float kc = det->keypoints[kp_id][2];
                if (kc < KP_CONF_THRESHOLD) continue;
                if (kx < 0 || kx >= src_img->width || ky < 0 || ky >= img_h) continue;

                // 2D pixel distance to nearest object
                float d2d = dist2D.at<float>((int)ky, (int)kx);
                if (d2d > SEARCH_RADIUS_PX) continue;

                // find exact nearest object pixel
                int rx = (int)(kx + 0.5f), ry = (int)(ky + 0.5f);
                int best_ox = -1, best_oy = -1;
                float best_d2 = 1e9f;
                int r = (int)(d2d + 4.0f);
                for (int dy = -r; dy <= r; dy++) {
                    int ny = ry + dy;
                    if (ny < 0 || ny >= img_h) continue;
                    for (int dx = -r; dx <= r; dx++) {
                        int nx = rx + dx;
                        if (nx < 0 || nx >= src_img->width) continue;
                        if (objMask.at<uint8_t>(ny, nx) == 0) continue;
                        float sd = sqrtf((float)(dx*dx + dy*dy));
                        if (sd < best_d2) { best_d2 = sd; best_ox = nx; best_oy = ny; }
                    }
                }

                if (best_ox < 0) continue;

                // 3D positions
                float Z_kp  = depthAt(depth_map, kx, ky);
                float Z_obj = depthAt(depth_map, (float)best_ox, (float)best_oy);
                if (Z_kp < 0 || Z_obj < 0) continue;

                Point3D kp3d, obj3d;
                pixelTo3D(kx, ky, Z_kp, fx, fy, cx, cy, kp3d);
                pixelTo3D((float)best_ox, (float)best_oy, Z_obj, fx, fy, cx, cy, obj3d);

                float d3d = dist3D(kp3d, obj3d);
                if (d3d < pi.min_dist_to_object) {
                    pi.min_dist_to_object = d3d;
                    pi.closest_kp_pt = kp3d;
                    pi.closest_obj_pt = obj3d;
                    pi.closest_kp_id = kp_id;
                }
            }
        }

        if (pi.min_dist_to_object < PROXIMITY_THRESHOLD_M)
            report->proximity_alert = true;
        if (pi.min_dist_to_object < report->min_distance_m)
            report->min_distance_m = pi.min_dist_to_object;

        report->persons.push_back(pi);
    }

    if (have_seg && !objMask.empty()) objMask.release();
    return 0;
}

bool should_save(const DangerReport& report)
{
    return report.balance_alert || report.climbing_alert || report.leaning_alert || report.proximity_alert;
}

static const char* kp_name(int id)
{
    switch (id) {
        case 0:  return "nose";
        case 9:  return "L-wrist";
        case 10: return "R-wrist";
        case 15: return "L-ankle";
        case 16: return "R-ankle";
        default: return "kp";
    }
}

void save_danger_frame(const DangerReport& report,
                       image_buffer_t* src_img,
                       cv::Mat& depth_map,
                       const std::string& out_dir)
{
    ensureDir(out_dir);

    char fname[256];
    snprintf(fname, sizeof(fname), "%s/frame_%05d", out_dir.c_str(), report.frame_id);

    // save annotated image (BGR → RGB → jpg via OpenCV)
    std::string img_path = std::string(fname) + ".jpg";
    {
        cv::Mat bgr(src_img->height, src_img->width, CV_8UC3);
        for (int y = 0; y < src_img->height; y++) {
            uint8_t* d = bgr.ptr<uint8_t>(y);
            unsigned char* s = src_img->virt_addr + y * src_img->width * 3;
            for (int x = 0; x < src_img->width; x++) {
                d[x*3+0] = s[x*3+2];
                d[x*3+1] = s[x*3+1];
                d[x*3+2] = s[x*3+0];
            }
        }
        cv::imwrite(img_path, bgr);
    }

    // save depth colormap
    std::string depth_path = std::string(fname) + "_depth.jpg";
    {
        cv::Mat dVis, dColor;
        dVis = (depth_map - 0.5) / 4.5 * 255.0;
        dVis.setTo(0, depth_map < 0.1f);
        dVis.convertTo(dVis, CV_8U);
        cv::applyColorMap(dVis, dColor, cv::COLORMAP_JET);
        dColor.setTo(cv::Scalar(0,0,0), depth_map < 0.1f);
        cv::imwrite(depth_path, dColor);
    }

    // save info txt
    std::string txt_path = std::string(fname) + ".txt";
    FILE* fp = fopen(txt_path.c_str(), "w");
    if (!fp) return;

    fprintf(fp, "Frame: %d\n", report.frame_id);
    fprintf(fp, "Alerts: BalanceLost=%d Climbing=%d Leaning=%d Proximity=%d\n",
            report.balance_alert, report.climbing_alert, report.leaning_alert, report.proximity_alert);
    fprintf(fp, "Min distance to object: %.3f m\n\n", report.min_distance_m);

    // objects
    fprintf(fp, "--- Dangerous objects (%zu) ---\n", report.object_names.size());
    for (size_t i = 0; i < report.object_names.size(); i++)
        fprintf(fp, "  [%zu] %s\n", i, report.object_names[i].c_str());
    fprintf(fp, "\n");

    // persons
    fprintf(fp, "--- Persons (%zu) ---\n", report.persons.size());
    for (size_t i = 0; i < report.persons.size(); i++) {
        const PersonDangerInfo& pi = report.persons[i];
        fprintf(fp, "Person %d:\n", pi.person_idx);
        fprintf(fp, "  Status: %s\n", status_label(pi.behavior));
        fprintf(fp, "  Tilt angle: %.1f deg\n", pi.balance_theta);
        fprintf(fp, "  Elbow: %.1f deg  Shoulder: %.1f deg  Knee: %.1f deg  Hip: %.1f deg\n",
                pi.elbow_angle, pi.shoulder_angle, pi.knee_angle, pi.hip_angle);
        fprintf(fp, "  Head-torso angle: %.1f deg  Nose-to-window: %.3f m\n",
                pi.head_torso_angle, pi.nose_to_window_dist);
        fprintf(fp, "  Min dist to object: %.3f m\n", pi.min_dist_to_object);
        fprintf(fp, "  Closest keypoint: %s (3D: %.3f %.3f %.3f)\n",
                kp_name(pi.closest_kp_id),
                pi.closest_kp_pt.X, pi.closest_kp_pt.Y, pi.closest_kp_pt.Z);
        fprintf(fp, "  Closest obj point: (3D: %.3f %.3f %.3f)\n\n",
                pi.closest_obj_pt.X, pi.closest_obj_pt.Y, pi.closest_obj_pt.Z);
    }

    fclose(fp);
    printf("[AutoSave] %s\n", fname);
}

std::string build_danger_json(const DangerReport& report)
{
    char buf[4096];
    int off = snprintf(buf, sizeof(buf),
        "{"
        "\"frame_id\":%d,"
        "\"balance_alert\":%s,"
        "\"climbing_alert\":%s,"
        "\"leaning_alert\":%s,"
        "\"proximity_alert\":%s,"
        "\"min_distance_m\":%.3f,"
        "\"cpu_usage\":%.1f,"
        "\"npu_usage\":%.1f,"
        "\"objects\":[",
        report.frame_id,
        report.balance_alert ? "true" : "false",
        report.climbing_alert ? "true" : "false",
        report.leaning_alert ? "true" : "false",
        report.proximity_alert ? "true" : "false",
        report.min_distance_m,
        report.cpu_usage,
        report.npu_usage);

    for (size_t i = 0; i < report.object_names.size(); i++) {
        if (i > 0) off += snprintf(buf + off, sizeof(buf) - off, ",");
        off += snprintf(buf + off, sizeof(buf) - off, "\"%s\"", report.object_names[i].c_str());
    }

    off += snprintf(buf + off, sizeof(buf) - off, "],\"persons\":[");
    for (size_t i = 0; i < report.persons.size(); i++) {
        if (i > 0) off += snprintf(buf + off, sizeof(buf) - off, ",");
        const PersonDangerInfo& p = report.persons[i];
        off += snprintf(buf + off, sizeof(buf) - off,
            "{"
            "\"idx\":%d,"
            "\"status\":\"%s\","
            "\"theta\":%.1f,"
            "\"elbow\":%.1f,"
            "\"shoulder\":%.1f,"
            "\"knee\":%.1f,"
            "\"hip\":%.1f,"
            "\"head_torso\":%.1f,"
            "\"nose_to_win\":%.3f,"
            "\"min_dist_m\":%.3f,"
            "\"kp_3d\":[%.3f,%.3f,%.3f],"
            "\"obj_3d\":[%.3f,%.3f,%.3f]"
            "}",
            p.person_idx,
            status_label(p.behavior),
            p.balance_theta,
            p.elbow_angle,
            p.shoulder_angle,
            p.knee_angle,
            p.hip_angle,
            p.head_torso_angle,
            p.nose_to_window_dist,
            p.min_dist_to_object,
            p.closest_kp_pt.X, p.closest_kp_pt.Y, p.closest_kp_pt.Z,
            p.closest_obj_pt.X, p.closest_obj_pt.Y, p.closest_obj_pt.Z);
    }
    off += snprintf(buf + off, sizeof(buf) - off, "]}");
    return std::string(buf);
}
