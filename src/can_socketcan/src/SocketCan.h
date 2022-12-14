/*
 * SocketCan.h
 *
 *  Created on: Mar 26, 2012
 *      Author: morl
 * 2016-03-08 Kjeld Jensen kjen@mmmi.sdu.dk Added support for sending extended id's
 */

#ifndef SOCKETCAN_H_
#define SOCKETCAN_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <can_socketcan/can.h>

#include <iostream>
#include <string>
#include <stdint.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

class SocketCan
{
public:
  SocketCan();
  virtual ~SocketCan();
  bool send_extended_id;
  bool flag_pub_active;
  
  int initInterface(std::string interface);
  void processCanTxEvent(const can_socketcan::can::ConstPtr& msg);

  ros::Publisher can_rx_publisher_;
  ros::Subscriber can_tx_subscriber_;

private:
  void canReadSome();
  void canRxHandler(const boost::system::error_code& error, size_t bytes_transferred);

  boost::asio::io_service ioService_;
  boost::asio::posix::stream_descriptor descriptor_;

  can_socketcan::can can_rx_msg_;

  int s;
  can_frame rx_;
};

#endif /* SOCKETCAN_H_ */
