#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <atomic>

#include "yolov8_pose.h"
#include "yolov8_seg.h"
#include "behavior_analyzer.h"
#include "danger_monitor.h"
#include "network_sender.h"
#include "sys_monitor.h"
#include "stereo_camera.h"
#include "image_utils.h"
#include "file_utils.h"
#include "image_drawing.h"
#include "image_convert.h"

static int skeleton[38] = {16, 14, 14, 12, 17, 15, 15, 13, 12, 13, 6, 12, 7, 13, 6, 7, 6, 8,
                           7, 9, 8, 10, 9, 11, 2, 3, 1, 2, 1, 3, 2, 4, 3, 5, 4, 6, 5, 7};

static unsigned char class_colors[][3] = {
    {255, 56, 56},   {255, 157, 151}, {255, 112, 31},  {255, 178, 29},  {207, 210, 49},
    {72, 249, 10},   {146, 204, 23},  {61, 219, 134},  {26, 147, 52},   {0, 212, 187},
    {44, 153, 168},  {0, 194, 255},   {52, 69, 147},   {100, 115, 255}, {0, 24, 236},
    {132, 56, 255},  {82, 0, 133},    {203, 56, 255},  {255, 149, 200}, {255, 55, 199}
};

static void draw_pose_results(image_buffer_t *img, pose_detect_result_list *pose_results,
                               int pose_ret, DangerReport *dangerRpt = nullptr)
{
    if (pose_ret != 0) return;

    printf("\n========== 姿态行为分析 ==========\n");
    char text[256];
    for (int i = 0; i < pose_results->count; i++) {
        pose_detect_result *det = &(pose_results->results[i]);

        BehaviorResult behavior;
        analyze_behavior(det, img->height, &behavior);

        // use danger report status if available (includes LEANING from window check)
        PoseStatus displayStatus = behavior.status;
        float noseToWin = -1.0f;
        if (dangerRpt && i < (int)dangerRpt->persons.size()) {
            displayStatus = dangerRpt->persons[i].behavior;
            noseToWin = dangerRpt->persons[i].nose_to_window_dist;
        }

        printf("[Person %d] 状态: %s | 倾角:%.1f° 肘:%.0f° 肩:%.0f° 膝:%.0f° 髋:%.0f° 头躯:%.0f° 鼻窗:%.2fm\n",
               i, status_to_string(displayStatus),
               behavior.balance_theta,
               behavior.elbow_angle, behavior.shoulder_angle,
               behavior.knee_angle, behavior.hip_angle,
               behavior.head_torso_angle, noseToWin);

        int x1 = det->box.left, y1 = det->box.top;
        int x2 = det->box.right, y2 = det->box.bottom;

        int box_color = COLOR_BLUE;
        if (displayStatus == STATUS_CLIMBING)
            box_color = COLOR_RED;
        else if (displayStatus == STATUS_BALANCE_LOST)
            box_color = COLOR_ORANGE;
        else if (displayStatus == STATUS_LEANING)
            box_color = 0xFFFF00FF; // magenta

        draw_rectangle(img, x1, y1, x2 - x1, y2 - y1, box_color, 3);

        sprintf(text, "%s | t:%.0f | E:%.0f S:%.0f K:%.0f H:%.0f",
                status_label(displayStatus), behavior.balance_theta,
                behavior.elbow_angle, behavior.shoulder_angle,
                behavior.knee_angle, behavior.hip_angle);
        draw_text(img, text, x1, y1 - 20, box_color, 10);

        for (int j = 0; j < 38 / 2; ++j) {
            draw_line(img,
                (int)(det->keypoints[skeleton[2*j]-1][0]),
                (int)(det->keypoints[skeleton[2*j]-1][1]),
                (int)(det->keypoints[skeleton[2*j+1]-1][0]),
                (int)(det->keypoints[skeleton[2*j+1]-1][1]),
                COLOR_ORANGE, 3);
        }
        for (int j = 0; j < 17; ++j) {
            draw_circle(img, (int)(det->keypoints[j][0]), (int)(det->keypoints[j][1]),
                        1, COLOR_YELLOW, 1);
        }

        if (behavior.center_x > 0 && behavior.foot_center_x > 0) {
            int cx = (int)behavior.center_x, cy = (int)behavior.center_y;
            int fx = (int)behavior.foot_center_x, fy = (int)behavior.foot_center_y;
            int lc = (behavior.status == STATUS_BALANCE_LOST) ? COLOR_RED : COLOR_GREEN;
            draw_circle(img, cx, cy, 5, COLOR_BLUE, -1);
            draw_circle(img, fx, fy, 5, COLOR_YELLOW, -1);
            draw_line(img, cx, cy, fx, fy, lc, 2);
        }

        int safe_y = (int)behavior.safe_height;
        draw_line(img, 0, safe_y, img->width, safe_y, COLOR_BLUE, 1);
    }
    printf("==================================\n\n");
}

static void draw_seg_results(image_buffer_t *img, seg_detect_result_list *seg_results, int seg_ret)
{
    if (seg_ret != 0) return;

    if (seg_results->count >= 1) {
        int w = img->width, h = img->height;
        char *ori = (char *)img->virt_addr;
        uint8_t *mask = seg_results->results_seg[0].seg_mask;
        float alpha = 0.5f;
        for (int j = 0; j < h; j++) {
            for (int k = 0; k < w; k++) {
                int off = 3 * (j * w + k);
                if (mask[j * w + k] != 0) {
                    int ci = mask[j * w + k] % N_CLASS_COLORS;
                    ori[off+0] = (unsigned char)clamp(class_colors[ci][0]*(1-alpha) + ori[off+0]*alpha, 0, 255);
                    ori[off+1] = (unsigned char)clamp(class_colors[ci][1]*(1-alpha) + ori[off+1]*alpha, 0, 255);
                    ori[off+2] = (unsigned char)clamp(class_colors[ci][2]*(1-alpha) + ori[off+2]*alpha, 0, 255);
                }
            }
        }
        // mask freed by caller after danger analysis
    }

    char text[256];
    for (int i = 0; i < seg_results->count; i++) {
        seg_detect_result *det = &(seg_results->results[i]);
        int x1 = det->box.left, y1 = det->box.top;
        int x2 = det->box.right, y2 = det->box.bottom;
        draw_rectangle(img, x1, y1, x2 - x1, y2 - y1, COLOR_RED, 3);
        sprintf(text, "%s %.1f%%", seg_cls_to_name(det->cls_id), det->prop * 100);
        draw_text(img, text, x1, y1 - 16, COLOR_BLUE, 10);
    }
}

static int run_image_mode(const char* image_path,
                           const char* pose_model_path,
                           const char* seg_model_path)
{

    rknn_pose_context_t pose_ctx;  memset(&pose_ctx, 0, sizeof(pose_ctx));
    rknn_seg_context_t  seg_ctx;   memset(&seg_ctx,  0, sizeof(seg_ctx));
    image_buffer_t src_image;      memset(&src_image, 0, sizeof(src_image));
    pose_detect_result_list pose_results; memset(&pose_results, 0, sizeof(pose_results));
    seg_detect_result_list  seg_results;  memset(&seg_results,  0, sizeof(seg_results));
    std::thread pose_thread, seg_thread;
    int pose_ret = 0, seg_ret = 0, ret;

    init_pose_post_process();
    init_seg_post_process();

    ret = init_yolov8_pose_model(pose_model_path, &pose_ctx);
    if (ret != 0) { printf("init pose fail\n"); goto out; }
    ret = init_yolov8_seg_model(seg_model_path, &seg_ctx);
    if (ret != 0) { printf("init seg fail\n"); goto out_pose; }

    ret = read_image(image_path, &src_image);
    if (ret != 0) { printf("read image fail\n"); goto out_seg; }

    pose_thread = std::thread([&](){ pose_ret = inference_yolov8_pose_model(&pose_ctx, &src_image, &pose_results); });
    seg_thread  = std::thread([&](){ seg_ret  = inference_yolov8_seg_model(&seg_ctx,  &src_image, &seg_results); });
    pose_thread.join();
    seg_thread.join();

    draw_seg_results(&src_image, &seg_results, seg_ret);
    draw_pose_results(&src_image, &pose_results, pose_ret);

    write_image("out.png", &src_image);
    printf("Result saved to out.png\n");

    if (seg_ret == 0 && seg_results.count >= 1 && seg_results.results_seg[0].seg_mask)
        free(seg_results.results_seg[0].seg_mask);
    if (src_image.virt_addr) free(src_image.virt_addr);
out_seg:
    release_yolov8_seg_model(&seg_ctx);
out_pose:
    release_yolov8_pose_model(&pose_ctx);
out:
    deinit_pose_post_process();
    deinit_seg_post_process();
    return 0;
}

static int run_camera_mode(int camId, const std::string& gstPipe,
                           const std::string& calibFile, float scale, bool fastMode,
                           const std::string& hostIp, int hostPort,
                           const char* pose_model_path,
                           const char* seg_model_path)
{
    StereoCamera stereo;
    if (stereo.init(calibFile, camId, gstPipe, scale, fastMode) != 0) {
        printf("stereo init fail\n");
        return -1;
    }

    rknn_pose_context_t pose_ctx; memset(&pose_ctx, 0, sizeof(pose_ctx));
    rknn_seg_context_t  seg_ctx;  memset(&seg_ctx,  0, sizeof(seg_ctx));

    init_pose_post_process();
    init_seg_post_process();

    if (init_yolov8_pose_model(pose_model_path, &pose_ctx) != 0) {
        printf("init pose fail\n");
        deinit_pose_post_process();
        deinit_seg_post_process();
        return -1;
    }
    if (init_yolov8_seg_model(seg_model_path, &seg_ctx) != 0) {
        printf("init seg fail\n");
        release_yolov8_pose_model(&pose_ctx);
        deinit_pose_post_process();
        deinit_seg_post_process();
        return -1;
    }

    const char* WIN = "YOLO Pose+Seg + Stereo Depth | ESC=Quit  AutoSave when DANGER";
    cv::namedWindow(WIN);

    NetworkSender sender;
    sender.connect(hostIp, hostPort);

    SysMonitor sysMon;

    // heartbeat thread — sends periodic keepalive to detect disconnect early
    std::atomic<bool> hbRunning{true};
    std::thread heartbeatThread([&]() {
        while (hbRunning) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            if (hbRunning && sender.isConnected())
                sender.sendHeartbeat();
        }
    });

    int frameCnt = 0;
    int reconnectAttempt = 0;

    while (true) {
        if (!stereo.grab()) break;

        image_buffer_t src_image;
        stereo.fillImageBuffer(&src_image);

        pose_detect_result_list pose_results; memset(&pose_results, 0, sizeof(pose_results));
        seg_detect_result_list  seg_results;  memset(&seg_results,  0, sizeof(seg_results));
        int pose_ret = 0, seg_ret = 0;

        auto t0 = std::chrono::steady_clock::now();

        std::thread pose_thread([&](){ pose_ret = inference_yolov8_pose_model(&pose_ctx, &src_image, &pose_results); });
        std::thread seg_thread([&](){  seg_ret  = inference_yolov8_seg_model(&seg_ctx,  &src_image, &seg_results); });
        pose_thread.join();
        seg_thread.join();

        auto t1 = std::chrono::steady_clock::now();
        float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

        // ---- danger analysis (must run before draw for LEANING status) ----
        cv::Mat depth = stereo.getDepth();
        DangerReport dangerRpt;
        dangerRpt.frame_id = frameCnt;
        analyze_danger(&pose_results, pose_ret, &seg_results, seg_ret,
                       depth, &src_image,
                       stereo.getFx(), stereo.getFy(), stereo.getCx(), stereo.getCy(),
                       &dangerRpt);

        draw_seg_results(&src_image, &seg_results, seg_ret);
        draw_pose_results(&src_image, &pose_results, pose_ret, &dangerRpt);

        // ---- auto-save + network send ----
        int w = src_image.width, hh = src_image.height;
        if (should_save(dangerRpt)) {
            save_danger_frame(dangerRpt, &src_image, depth, "outputimage");
            printf("[AutoSave] frame %d | balance=%d climb=%d lean=%d prox=%.2fm\n",
                   frameCnt, dangerRpt.balance_alert,
                   dangerRpt.climbing_alert, dangerRpt.leaning_alert,
                   dangerRpt.min_distance_m);

            // encode annotated image to JPEG
            cv::Mat annBGR = rgb_buffer_to_bgr_mat(&src_image);
            std::vector<uchar> jpg1, jpg2;
            cv::imencode(".jpg", annBGR, jpg1);

            // depth colormap
            cv::Mat dVis, dColor;
            dVis = (depth - 0.5) / 4.5 * 255.0;
            dVis.setTo(0, depth < 0.1f);
            dVis.convertTo(dVis, CV_8U);
            cv::applyColorMap(dVis, dColor, cv::COLORMAP_JET);
            dColor.setTo(cv::Scalar(0,0,0), depth < 0.1f);
            cv::imencode(".jpg", dColor, jpg2);

            sysMon.sample();
            dangerRpt.cpu_usage = sysMon.cpuUsage();
            dangerRpt.npu_usage = sysMon.npuUsage();

            std::string json = build_danger_json(dangerRpt);
            if (sender.sendFrame(json,
                    jpg1.data(), (uint32_t)jpg1.size(),
                    jpg2.data(), (uint32_t)jpg2.size()) != 0) {
                // reconnect on failure
                if (++reconnectAttempt % 10 == 0) {
                    printf("[Reconnect] attempting to reconnect to %s:%d...\n", hostIp.c_str(), hostPort);
                    sender.connect(hostIp, hostPort);
                }
            } else {
                reconnectAttempt = 0;
            }
        }

        // ---- free seg mask ----
        if (seg_ret == 0 && seg_results.count >= 1 && seg_results.results_seg[0].seg_mask) {
            free(seg_results.results_seg[0].seg_mask);
            seg_results.results_seg[0].seg_mask = nullptr;
        }

        // ---- build display ----
        cv::Mat display = rgb_buffer_to_bgr_mat(&src_image);

        // FPS
        char fpsText[64];
        snprintf(fpsText, sizeof(fpsText), "%.1f ms (%.1f FPS)", ms, 1000.f/ms);
        cv::putText(display, fpsText, cv::Point(5, hh - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

        // depth color map
        cv::Mat depthVis, depthColor;
        if (!depth.empty()) {
            depthVis = (depth - 0.5) / 4.5 * 255.0;
            depthVis.setTo(0, depth < 0.1f);
            depthVis.convertTo(depthVis, CV_8U);
            cv::applyColorMap(depthVis, depthColor, cv::COLORMAP_JET);
            depthColor.setTo(cv::Scalar(0,0,0), depth < 0.1f);
        } else {
            depthColor = cv::Mat::zeros(hh, w, CV_8UC3);
        }

        // status banner — use danger report for correct status (incl. LEANING)
        if (pose_ret == 0 && pose_results.count > 0 && !dangerRpt.persons.empty()) {
            BehaviorResult bh;
            analyze_behavior(&pose_results.results[0], hh, &bh);
            PoseStatus st = dangerRpt.persons[0].behavior;
            const char* stLabel = status_label(st);
            cv::Scalar bannerColor = (st == STATUS_NORMAL) ? cv::Scalar(0,255,0) :
                                     (st == STATUS_CLIMBING) ? cv::Scalar(0,0,255) :
                                     (st == STATUS_LEANING)  ? cv::Scalar(255,0,255) :
                                                               cv::Scalar(0,165,255);
            char buf[256];
            snprintf(buf, sizeof(buf), "%s | t:%.0f E:%.0f S:%.0f K:%.0f H:%.0f HT:%.0f | dist=%.2fm",
                     stLabel, bh.balance_theta,
                     bh.elbow_angle, bh.shoulder_angle,
                     bh.knee_angle, bh.hip_angle,
                     bh.head_torso_angle,
                     dangerRpt.min_distance_m);
            cv::putText(display, buf, cv::Point(5, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, bannerColor, 1);
        }

        // proximity alert overlay
        if (dangerRpt.proximity_alert) {
            cv::putText(display, "!! TOO CLOSE !!",
                        cv::Point(w/2 - 80, hh/2),
                        cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 3);
        }

        cv::Mat combo;
        cv::hconcat(display, depthColor, combo);
        cv::imshow(WIN, combo);

        free(src_image.virt_addr);
        ++frameCnt;

        int key = cv::waitKey(1) & 0xFF;
        if (key == 27) break;
    }

    hbRunning = false;
    if (heartbeatThread.joinable())
        heartbeatThread.join();

    cv::destroyAllWindows();
    release_yolov8_seg_model(&seg_ctx);
    release_yolov8_pose_model(&pose_ctx);
    deinit_pose_post_process();
    deinit_seg_post_process();
    stereo.release();
    return 0;
}

int main(int argc, char **argv)
{
    setenv("NO_AT_BRIDGE", "1", 1);

    std::string imagePath;
    std::string gstPipe;
    std::string calibFile  = "stereo_calib.yaml";
    std::string poseModel  = "model/yolov8_pose.rknn";
    std::string segModel   = "model/yolov8_seg.rknn";
    std::string hostIp     = "192.168.137.1";
    int   camId    = 41;
    int   hostPort = 9527;
    float scale    = 0.5f;
    bool  fastMode = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--image")                imagePath = argv[++i];
        else if (a == "-c" || a == "--camera")   camId = std::stoi(argv[++i]);
        else if (a == "-p" || a == "--pipeline") gstPipe = argv[++i];
        else if (a == "-f" || a == "--calib")   calibFile = argv[++i];
        else if (a == "-s" || a == "--scale")   scale = std::stof(argv[++i]);
        else if (a == "--fast")                  fastMode = true;
        else if (a == "--host")                  hostIp = argv[++i];
        else if (a == "--port")                  hostPort = std::stoi(argv[++i]);
        else if (a == "--pose-model")            poseModel = argv[++i];
        else if (a == "--seg-model")             segModel = argv[++i];
        else if (a == "-h" || a == "--help") {
            printf("Usage: %s [--image <path>] [options]\n", argv[0]);
            printf("  --image <path>      Single image mode\n");
            printf("  --pose-model <path> Pose model path (default: model/yolov8_pose.rknn)\n");
            printf("  --seg-model <path>  Seg model path (default: model/yolov8_seg.rknn)\n");
            printf("  -c, --camera <id>   Camera device ID (default: 41)\n");
            printf("  -p, --pipeline      GStreamer pipeline\n");
            printf("  -f, --calib <f>     Calibration YAML (default: stereo_calib.yaml)\n");
            printf("  -s, --scale <f>     SGBM scale (default: 0.5)\n");
            printf("  --fast              Skip WLS filter\n");
            printf("  --host <ip>         PC server IP (default: 192.168.137.1)\n");
            printf("  --port <port>       PC server port (default: 9527)\n");
            printf("  -h, --help          This help\n");
            return 0;
        } else {
            imagePath = a; // legacy: first positional arg = image path
        }
    }

    if (!imagePath.empty()) {
        return run_image_mode(imagePath.c_str(), poseModel.c_str(), segModel.c_str());
    } else {
        return run_camera_mode(camId, gstPipe, calibFile, scale, fastMode, hostIp, hostPort,
                               poseModel.c_str(), segModel.c_str());
    }
}
