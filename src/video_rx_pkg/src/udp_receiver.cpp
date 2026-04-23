#include "udp_receiver.h"
#include <iostream>
#include <cstring>

namespace asio = boost::asio;
using     udp  = asio::ip::udp;

UdpReceiver::UdpReceiver(asio::io_context& ioc,
                         uint16_t          port,
                         FrameAssembler&   assembler)
    : socket_(ioc, udp::endpoint(udp::v4(), port))
    , assembler_(assembler)
    , port_(port)
{
    asio::socket_base::receive_buffer_size opt(4 * 1024 * 1024);
    socket_.set_option(opt);
    std::cout << "[udp_receiver] listening on port " << port << "\n";
}

void UdpReceiver::start() { do_receive(); }
void UdpReceiver::stop()  { socket_.close(); }

void UdpReceiver::do_receive()
{
    socket_.async_receive_from(
        asio::buffer(buf_), remote_,
        [this](boost::system::error_code ec, std::size_t bytes) {
            if (ec) {
                if (ec != asio::error::operation_aborted)
                    std::cerr << "[udp_receiver] recv error: " << ec.message() << "\n";
                return;
            }

            if (bytes < HEADER_SIZE) {
                std::cerr << "[udp_receiver] packet too small: " << bytes << "\n";
                do_receive();
                return;
            }

            CamHeader hdr;
            std::memcpy(&hdr, buf_.data(), HEADER_SIZE);

            // magic 먼저 확인
            if (!cam_header_valid(hdr)) {
                std::cerr << "[udp_receiver] invalid magic, dropping\n";
                do_receive();
                return;
            }

            const uint8_t* payload     = buf_.data() + HEADER_SIZE;
            const uint16_t payload_len = static_cast<uint16_t>(bytes - HEADER_SIZE);

            // ✅ ntohs() 적용해서 비교
            if (payload_len != cam_header_size(hdr)) {
                std::cerr << "[udp_receiver] size mismatch: hdr.size="
                          << cam_header_size(hdr)
                          << " actual=" << payload_len << "\n";
                do_receive();
                return;
            }
            /*
            // ✅ 헬퍼 함수로 엔디안 변환된 값 출력
            std::cout << "[udp] frame=" << cam_header_frame_id(hdr)
                      << " offset="     << cam_header_offset(hdr)
                      << " size="       << payload_len
                      << " res="        << cam_header_width(hdr)
                      << "x"            << cam_header_height(hdr) << "\n";
            */
            assembler_.push_chunk(hdr, payload, payload_len);
            do_receive();
        });
}