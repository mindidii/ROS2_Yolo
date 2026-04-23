#pragma once
#include "cam_header.h"
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <chrono>

// ─────────────────────────────────────────────────────────────
//  완성된 프레임 정보
// ─────────────────────────────────────────────────────────────
struct AssembledFrame {
    uint8_t  camera_id;
    uint32_t frame_id;
    uint16_t width;
    uint16_t height;
    uint64_t timestamp_ms;
    std::vector<uint8_t> data;   // 완전한 프레임 raw bytes
};

// 완성된 프레임을 받을 콜백 타입
// AI 추론 등 downstream 처리는 여기에 연결
using FrameCallback = std::function<void(AssembledFrame)>;

// ─────────────────────────────────────────────────────────────
//  FrameAssembler
//  - 카메라 ID별로 독립된 재조립 상태 유지
//  - 청크가 모두 수신되면 FrameCallback 호출
//  - 오래된 미완성 프레임 자동 정리 (stale_ms 이후)
// ─────────────────────────────────────────────────────────────
class FrameAssembler {
public:
    explicit FrameAssembler(FrameCallback cb,
                            uint32_t stale_ms = 2000);

    // 수신된 패킷 1개를 처리 (헤더 + payload)
    void push_chunk(const CamHeader& header,
                    const uint8_t*  payload,
                    uint16_t        payload_len);

    // 오래된 미완성 프레임 정리 (주기적으로 호출)
    void purge_stale();

    // 수신 통계
    struct Stats {
        uint64_t packets_received  = 0;
        uint64_t frames_completed  = 0;
        uint64_t frames_dropped    = 0;
        uint64_t bytes_received    = 0;
    };
    Stats stats() const;

private:
    struct FrameSlot {
        uint16_t width  = 0;
        uint16_t height = 0;
        uint64_t timestamp_ms = 0;
        uint32_t received_bytes = 0;
        uint32_t expected_bytes = 0;   // 첫 청크 수신 후 결정
        bool     size_known = false;
        std::vector<uint8_t> buffer;
        std::chrono::steady_clock::time_point last_update;
    };

    using FrameKey = uint64_t; // (camera_id << 32) | frame_id
    static FrameKey make_key(uint8_t cam, uint32_t fid) {
        return (static_cast<uint64_t>(cam) << 32) | fid;
    }

    FrameCallback  cb_;
    uint32_t       stale_ms_;
    mutable std::mutex         mu_;
    std::unordered_map<FrameKey, FrameSlot> slots_;
    Stats stats_;
};