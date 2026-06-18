#include "network_sender.h"
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/tcp.h>

NetworkSender::NetworkSender() : m_sock(-1) {}

NetworkSender::~NetworkSender() { disconnect(); }

int NetworkSender::connect(const std::string& host, int port)
{
    disconnect();

    m_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (m_sock < 0) {
        perror("[NetSender] socket");
        return -1;
    }

    int flag = 1;
    setsockopt(m_sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) <= 0) {
        perror("[NetSender] inet_pton");
        disconnect();
        return -1;
    }

    // non-blocking connect with 3-second timeout
    int flags = fcntl(m_sock, F_GETFL, 0);
    fcntl(m_sock, F_SETFL, flags | O_NONBLOCK);

    int ret = ::connect(m_sock, (struct sockaddr*)&addr, sizeof(addr));
    if (ret < 0 && errno != EINPROGRESS) {
        perror("[NetSender] connect");
        disconnect();
        return -1;
    }

    if (ret == 0) {
        // connected immediately (localhost)
        fcntl(m_sock, F_SETFL, flags);
        printf("[NetSender] connected to %s:%d\n", host.c_str(), port);
        return 0;
    }

    // wait for connection with timeout
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(m_sock, &wfds);
    struct timeval tv = {3, 0}; // 3 seconds

    ret = select(m_sock + 1, NULL, &wfds, NULL, &tv);
    if (ret <= 0) {
        printf("[NetSender] connect to %s:%d timed out (will run offline)\n", host.c_str(), port);
        disconnect();
        return -1;
    }

    // check if connection succeeded
    int err = 0;
    socklen_t len = sizeof(err);
    getsockopt(m_sock, SOL_SOCKET, SO_ERROR, &err, &len);
    if (err != 0) {
        printf("[NetSender] connect to %s:%d failed (will run offline)\n", host.c_str(), port);
        disconnect();
        return -1;
    }

    fcntl(m_sock, F_SETFL, flags); // restore blocking mode
    printf("[NetSender] connected to %s:%d\n", host.c_str(), port);
    return 0;
}

void NetworkSender::disconnect()
{
    if (m_sock >= 0) {
        close(m_sock);
        m_sock = -1;
    }
}

int NetworkSender::sendAll(const void* data, uint32_t len)
{
    const char* p = (const char*)data;
    uint32_t remaining = len;
    while (remaining > 0) {
        ssize_t n = send(m_sock, p, remaining, 0);
        if (n <= 0) return -1;
        remaining -= n;
        p += n;
    }
    return 0;
}

int NetworkSender::sendU32(uint32_t v)
{
    uint32_t nv = htonl(v);
    return sendAll(&nv, 4);
}

int NetworkSender::sendFrame(const std::string& json,
                             const uint8_t* img1, uint32_t len1,
                             const uint8_t* img2, uint32_t len2)
{
    if (m_sock < 0) return -1;

    uint32_t jsonLen = (uint32_t)json.size();
    if (sendU32(jsonLen) != 0)  return -1;
    if (sendAll(json.data(), jsonLen) != 0) return -1;

    if (sendU32(len1) != 0)     return -1;
    if (len1 > 0 && sendAll(img1, len1) != 0) return -1;

    if (sendU32(len2) != 0)     return -1;
    if (len2 > 0 && sendAll(img2, len2) != 0) return -1;

    return 0;
}

int NetworkSender::sendHeartbeat()
{
    if (m_sock < 0) return -1;
    uint32_t jsonLen = 2;
    if (sendU32(jsonLen) != 0) return -1;
    if (sendAll("{}", 2) != 0) return -1;
    if (sendU32(0) != 0) return -1;
    if (sendU32(0) != 0) return -1;
    return 0;
}
