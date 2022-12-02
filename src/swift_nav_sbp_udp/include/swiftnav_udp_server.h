#ifndef SWIFTNAV_UDP_SERVER_H
#define SWIFTNAV_UDP_SERVER_H

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <memory>
#include <functional>
#include "libsbp/edc.h"


using boost::asio::ip::udp;

#pragma pack(push, 1)
struct SbpMsgHeader
{
    std::uint8_t Preamble;
    std::uint16_t MsgType;
    std::uint16_t SenderId;
    std::uint8_t Length;
};
#pragma pack(pop)

typedef std::function<void (SbpMsgHeader, std::vector<std::uint8_t>, boost::asio::ip::address)> t_MsgCallback;
typedef std::map<std::uint16_t, t_MsgCallback> t_mapIdToMsgCallback;

class SwiftNavUdpServer
{
public:
    SwiftNavUdpServer(boost::asio::io_service& io_service, unsigned short port, const t_MsgCallback &handler)
        : socket_(io_service, udp::endpoint(udp::v4(), port)), callbackHandler(handler)
    {
        start_receive();
    }

    bool isError(){return _isError;}

private:
    void start_receive()
    {
        socket_.async_receive_from(
                    boost::asio::buffer(recv_buffer_), remote_endpoint_,
                    boost::bind(&SwiftNavUdpServer::handle_receive, this,
                                boost::asio::placeholders::error,
                                boost::asio::placeholders::bytes_transferred));
    }

    void handle_receive(const boost::system::error_code& error,
                        std::size_t bytes_transferred)
    {
        if (!error || error == boost::asio::error::message_size)
        {
            SbpMsgHeader header;
            std::vector<std::uint8_t> payload(512);
            std::uint16_t CRC = 0;
            std::uint16_t calculatedCRC = 0;

            if (bytes_transferred <= 8)
            {
                ROS_ERROR("Received empty or invalid sbp frame with size: %lu", bytes_transferred);
                goto wait_for_new_message;
            }

            memcpy(&header, recv_buffer_.data(), 6);
            if(header.Preamble != 0x55)
            {
                ROS_ERROR("Received invalid sbp frame with invalid preamble: %d", header.Preamble);
                goto wait_for_new_message;
            }
            if(header.Length != (bytes_transferred - 8))
            {
                ROS_ERROR("Received invalid sbp frame with invalid length: %d udp size: %lu but ignoring", header.Length, bytes_transferred);
                goto wait_for_new_message;
            }

            payload.resize(header.Length);
            memcpy(payload.data(), recv_buffer_.data() + 6, header.Length);

            memcpy(&CRC, recv_buffer_.data() + (bytes_transferred - 2), sizeof(CRC));

            calculatedCRC = crc16_ccitt((u8*)&(header.MsgType), 2, 0);
            calculatedCRC = crc16_ccitt((u8*)&(header.SenderId), 2, calculatedCRC);
            calculatedCRC = crc16_ccitt(&(header.Length), 1, calculatedCRC);
            calculatedCRC = crc16_ccitt(payload.data(), header.Length, calculatedCRC);

            if (CRC != calculatedCRC)
            {
                ROS_ERROR("Received invalid sbp frame with invalid CRC: %d calculated %d", CRC, calculatedCRC);
                goto wait_for_new_message;
            }

            //ROS_INFO("Received message type %d from: %s", header.MsgType, remote_endpoint_.address().to_string().c_str());
            callbackHandler(header, payload, remote_endpoint_.address());


            wait_for_new_message:
            start_receive();
        }
        else
        {
            ROS_ERROR("Error during reading imu udp frame: %s", error.message().c_str());
            _isError = true;
        }
    }

    udp::socket socket_;
    udp::endpoint remote_endpoint_;
    boost::array<char, 512> recv_buffer_;
    bool _isError = false;
    t_MsgCallback callbackHandler;
};

#endif // SWIFTNAV_UDP_SERVER_H
