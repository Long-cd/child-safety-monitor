#ifndef _SYS_MONITOR_H_
#define _SYS_MONITOR_H_

class SysMonitor {
public:
    SysMonitor();

    // call once per frame; returns 0 on success
    int sample();

    float cpuUsage() const { return m_cpu; }
    float npuUsage() const { return m_npu; }

private:
    float  m_cpu;
    float  m_npu;
    unsigned long long m_prevTotal;
    unsigned long long m_prevIdle;
    bool   m_first;
};

#endif
