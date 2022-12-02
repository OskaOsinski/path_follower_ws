#include <string>
#include <thread>
#include <memory>
#include <functional>
#include <ros/ros.h>
#include "swiftnav_udp_server.h"
#include "gps_conv.h"
#include "position_handler.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "swift_nav_sbp_udp");
    ros::NodeHandlePtr publicNode(new ros::NodeHandle());
    ros::NodeHandlePtr privateNode(new ros::NodeHandle("~"));

    int serverPort = 9865;
    privateNode->param<int>("udp_port", serverPort, 9865);

    PositionHandler posHandler(publicNode, privateNode);
    std::thread ioThread([&]()
    {
        ROS_INFO("IMU UDP server enter %d", serverPort);
        while(! ros::isShuttingDown())
        {
            boost::asio::io_service io_service;
            SwiftNavUdpServer server(io_service, static_cast<unsigned short>(serverPort), posHandler.getCallbackHandler());
            while ( !ros::isShuttingDown() && !server.isError())
            {
                try
                {
                    io_service.run_one();
                }
                catch (std::exception& e)
                {
                    ROS_FATAL("%s", e.what());
                }
            }
            ROS_INFO("IMU UDP server inner loop end");
        }
        ROS_INFO("IMU UDP server exit");
    });

    ros::spin();
    ioThread.join();
    ROS_INFO("IMU UDP server joined");

    return 0;
}

