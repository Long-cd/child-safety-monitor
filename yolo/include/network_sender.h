#ifndef _NETWORK_SENDER_H_
#define _NETWORK_SENDER_H_

#include <string>
#include <cstdint>

class NetworkSender {
public:
    NetworkSender();
    ~NetworkSender();

    int  connect(const std::string& host, int port);
    void disconnect();
    bool isConnected() const { return m_sock >= 0; }

    // send JSON metadata + 2 JPEG images (annotated + depth)
    // images are JPEG-encoded byte buffers
    int  sendFrame(const std::string& json,
                   const uint8_t* img1, uint32_t len1,
                   const uint8_t* img2, uint32_t len2);

    // lightweight heartbeat to detect disconnect
    int  sendHeartbeat();

private:
    int  m_sock;
    int  sendAll(const void* data, uint32_t len);
    int  sendU32(uint32_t v);
};

#endif
