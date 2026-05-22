#pragma once
#include <cstdint>
#include <cstring>
#include <arpa/inet.h>   // ntohl, ntohs

// ─── YUYV 송신 측(yuyv-udp-stream) 헤더와 동일 ───────────────
static constexpr uint16_t MAX_PAYLOAD  = 1400;
static constexpr uint16_t HEADER_SIZE  = 20;

static constexpr uint8_t  CAM_ID_PCAM  = 0x01;   // 추후 camera_id 구분용
static constexpr uint8_t  CAM_ID_EO    = CAM_ID_PCAM;
static constexpr uint8_t  CAM_ID_IR    = 0x02;

#pragma pack(push, 1)
struct CamHeader {
    char     magic[4];      // "YUYV"
    uint32_t frame_id;      // big-endian
    uint32_t offset;        // big-endian, byte offset
    uint16_t size;          // big-endian, payload bytes
    uint16_t flags;         // big-endian, bit0=first_chunk, bit1=camera_id(0=PCAM,1=IR)
    uint32_t timestamp_ms;  // big-endian, milliseconds since boot
};
#pragma pack(pop)

static_assert(sizeof(CamHeader) == HEADER_SIZE, "CamHeader must be exactly 20 bytes");

// ─── 파싱 헬퍼 (big-endian → host) ───────────────────────────
inline bool cam_header_valid(const CamHeader& h) {
    return std::memcmp(h.magic, "YUYV", 4) == 0;
}

inline uint32_t cam_header_frame_id(const CamHeader& h) { return ntohl(h.frame_id); }
inline uint32_t cam_header_offset  (const CamHeader& h) { return ntohl(h.offset);   }
inline uint16_t cam_header_size        (const CamHeader& h) { return ntohs(h.size);          }
inline uint16_t cam_header_flags       (const CamHeader& h) { return ntohs(h.flags);         }
inline uint32_t cam_header_timestamp_ms(const CamHeader& h) { return ntohl(h.timestamp_ms);  }
inline bool     cam_header_first_chunk (const CamHeader& h) { return (cam_header_flags(h) & 0x0001) != 0; }
inline uint8_t  cam_header_camera_id   (const CamHeader& h) { return (cam_header_flags(h) & 0x0002) ? CAM_ID_IR : CAM_ID_PCAM; }
