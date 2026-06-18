#include "sys_monitor.h"
#include <cstdio>
#include <cstring>

static const char* NPU_PATHS[] = {
    "/sys/devices/platform/fdab0000.npu/devfreq/fdab0000.npu/load",
    "/sys/kernel/debug/rknpu/load",
    "/sys/class/devfreq/fdab0000.npu/load",
    "/sys/devices/platform/fdab0000.npu/load",
    NULL
};

SysMonitor::SysMonitor() : m_cpu(0), m_npu(0),
    m_prevTotal(0), m_prevIdle(0), m_first(true) {}

int SysMonitor::sample()
{
    // ── CPU usage ──
    FILE* fp = fopen("/proc/stat", "r");
    if (!fp) return -1;

    char buf[256];
    fgets(buf, sizeof(buf), fp);
    fclose(fp);

    unsigned long long user, nice, system, idle, iowait, irq, softirq, steal;
    int n = sscanf(buf, "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                   &user, &nice, &system, &idle, &iowait, &irq, &softirq, &steal);
    if (n < 4) return -1;

    unsigned long long total = user + nice + system + idle + iowait + irq + softirq + steal;
    unsigned long long idle_total = idle + iowait;

    if (!m_first) {
        unsigned long long dTotal = total - m_prevTotal;
        unsigned long long dIdle  = idle_total - m_prevIdle;
        if (dTotal > 0)
            m_cpu = 100.0f * (1.0f - (float)dIdle / (float)dTotal);
        else
            m_cpu = 0;
    }
    m_prevTotal = total;
    m_prevIdle  = idle_total;
    m_first = false;

    // ── NPU usage ──
    m_npu = -1.0f;
    for (int i = 0; NPU_PATHS[i] != NULL; i++) {
        fp = fopen(NPU_PATHS[i], "r");
        if (fp) {
            int load;
            if (fscanf(fp, "%d", &load) == 1)
                m_npu = (float)load;
            fclose(fp);
            break;
        }
    }

    return 0;
}