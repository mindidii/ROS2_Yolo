#include "frame_assembler.h"
#include <iostream>
#include <cstring>
#include <optional>
#include <arpa/inet.h>

FrameAssembler::FrameAssembler(FrameCallback cb, uint32_t stale_ms)
    : cb_(std::move(cb)), stale_ms_(stale_ms) {}

void FrameAssembler::push_chunk(const CamHeader& hdr,
                                const uint8_t*   payload,
                                uint16_t         payload_len)
{
    if (!cam_header_valid(hdr)) {
        std::cerr << "[assembler] invalid magic, dropping packet\n";
        return;
    }
    if (payload_len == 0 || payload_len > MAX_PAYLOAD) {
        std::cerr << "[assembler] invalid payload_len=" << payload_len << "\n";
        return;
    }

    const uint32_t frame_id    = cam_header_frame_id(hdr);
    const uint32_t offset      = cam_header_offset(hdr);
    const uint32_t timestamp   = cam_header_timestamp_ms(hdr);
    const uint8_t  camera_id   = cam_header_camera_id(hdr);
    const bool     first_chunk = cam_header_first_chunk(hdr);

    uint16_t width = 0, height = 0;
    if (first_chunk && payload_len >= 4) {
        width  = ntohs(*(const uint16_t *)(payload + 0));
        height = ntohs(*(const uint16_t *)(payload + 2));
    }

    const FrameKey key = make_key(camera_id, frame_id);
    const uint32_t end = offset + payload_len;

    std::optional<AssembledFrame> completed;

    {
        std::lock_guard<std::mutex> lk(mu_);
        stats_.packets_received++;
        stats_.bytes_received += payload_len;

        auto& slot = slots_[key];

        if (slot.buffer.empty()) {
            slot.timestamp_ms = timestamp;
            slot.last_update  = std::chrono::steady_clock::now();
        }

        if (first_chunk && payload_len >= 4) {
            slot.width  = width;
            slot.height = height;
        }

        if (end > slot.buffer.size())
            slot.buffer.resize(end, 0);

        std::memcpy(slot.buffer.data() + offset, payload, payload_len);
        slot.received_bytes += payload_len;
        slot.last_update = std::chrono::steady_clock::now();

        if (payload_len < MAX_PAYLOAD) {
            slot.size_known     = true;
            slot.expected_bytes = end;
        }

        if (slot.size_known && slot.received_bytes >= slot.expected_bytes) {
            AssembledFrame frame;
            frame.camera_id    = camera_id;
            frame.frame_id     = frame_id;
            frame.width        = slot.width;
            frame.height       = slot.height;
            frame.timestamp_ms = slot.timestamp_ms;
            frame.data         = std::move(slot.buffer);
            slots_.erase(key);
            stats_.frames_completed++;
            completed = std::move(frame);
        }
    }

    if (completed)
        cb_(std::move(*completed));
}

void FrameAssembler::purge_stale()
{
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lk(mu_);
    for (auto it = slots_.begin(); it != slots_.end(); ) {
        auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now - it->second.last_update).count();
        if (age > stale_ms_) {
            std::cerr << "[assembler] stale frame dropped: cam="
                      << static_cast<int>(it->first >> 32)
                      << " frame_id=" << (it->first & 0xFFFFFFFF)
                      << " width=" << it->second.width
                      << " height=" << it->second.height
                      << " received=" << it->second.received_bytes
                      << " expected=" << it->second.expected_bytes
                      << " size_known=" << it->second.size_known
                      << " age=" << age << "ms\n";
            stats_.frames_dropped++;
            it = slots_.erase(it);
        } else {
            ++it;
        }
    }
}

FrameAssembler::Stats FrameAssembler::stats() const
{
    std::lock_guard<std::mutex> lk(mu_);
    return stats_;
}
