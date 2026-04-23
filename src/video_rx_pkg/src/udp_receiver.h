#pragma once
#include "frame_assembler.h"
#include <boost/asio.hpp>
#include <thread>
#include <vector>
#include <atomic>

// ─────────────────────────────────────────────────────────────
//  UdpReceiver
//  - 지정된 포트에서 비동기 UDP 수신
//  - 수신된 패킷을 FrameAssembler 로 전달
//  - 포트당 1개 인스턴스 (video0=5000, video1=5001)
// ─────────────────────────────────────────────────────────────
class UdpReceiver {
public:
    UdpReceiver(boost::asio::io_context& ioc,
                uint16_t                 port,
                FrameAssembler&          assembler);

    void start();
    void stop();

private:
    void do_receive();

    boost::asio::ip::udp::socket   socket_;
    boost::asio::ip::udp::endpoint remote_;
    FrameAssembler&                assembler_;
    uint16_t port_;
    
    // 최대 패킷 크기: 헤더 26 + payload 1400 + 여유 64
    static constexpr size_t BUF_SIZE = HEADER_SIZE + MAX_PAYLOAD + 64;
    std::array<uint8_t, BUF_SIZE>  buf_;
};