/*
 *  P2OS for ROS
 *  Copyright (C) 2000  David Feil-Seifer, Brian Gerkey, Kasper Stoy,
 *      Richard Vaughan, & Andrew Howard
 *  Copyright (C) 2018  Hunter L. Allen
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <p2os_driver/packet.hpp>
#include <unistd.h>
#include <stdlib.h>
#include <sys/select.h>
#include "rclcpp/rclcpp.hpp"

// Wait for data on fd with timeout. Returns 1 if ready, 0 if timeout, -1 on error.
static int wait_for_data(int fd, int timeout_ms)
{
  fd_set rfds;
  struct timeval tv;
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  return select(fd + 1, &rfds, NULL, NULL, &tv);
}

void P2OSPacket::Print()
{
  if (packet) {
    RCLCPP_INFO(logger_, "\"");
    for (int i = 0; i < size; i++) {
      RCLCPP_INFO(logger_, "%u ", packet[i]);
    }
    RCLCPP_INFO(logger_, "\"");
  }
}

void P2OSPacket::PrintHex()
{
  if (packet) {
    RCLCPP_INFO(logger_, "\"");
    for (int i = 0; i < size; i++) {
      RCLCPP_INFO(logger_, "0x%.2x ", packet[i]);
    }
    RCLCPP_INFO(logger_, "\"");
  }
}


bool P2OSPacket::Check()
{
  const int16_t chksum = CalcChkSum();
  return (chksum == (packet[size - 2] << 8)) | packet[size - 1];
}

int P2OSPacket::CalcChkSum()
{
  unsigned char * buffer = &packet[3];
  int c = 0;
  int n;

  for (n = size - 5; n > 1; ) {
    c += (*(buffer) << 8) | *(buffer + 1);
    c = c & 0xffff;
    n -= 2;
    buffer += 2;
  }
  if (n > 0) {
    c ^= static_cast<int>(*(buffer++));
  }

  return c;
}

int P2OSPacket::Receive(int fd, int timeout_ms)
{
  unsigned char prefix[3];
  int cnt;

  ::memset(packet, 0, sizeof(packet));

  do {
    ::memset(prefix, 0, sizeof(prefix));

    while (1) {
      if (timeout_ms > 0) {
        // Interruptible mode: use select() so we can check for shutdown
        if (!rclcpp::ok()) {
          return 2;
        }
        int ready = wait_for_data(fd, timeout_ms);
        if (ready == 0) { continue; }  // timeout, loop to check rclcpp::ok()
        if (ready < 0) {
          if (errno == EINTR) { continue; }
          RCLCPP_ERROR(logger_, "select() error in Receive(): %s", strerror(errno));
          return 1;
        }
      }

      cnt = read(fd, &prefix[2], 1);
      if (cnt < 0) {
        int saved_errno = errno;
        if (saved_errno == EINTR) { continue; }
        // EAGAIN is expected when timeout_ms=0 (non-blocking SYNC); retry when using select()
        if ((saved_errno == EAGAIN || saved_errno == EWOULDBLOCK) && timeout_ms > 0) { continue; }
        // During SYNC0 the fd is O_NONBLOCK, so EAGAIN just means no data yet — not an error
        if (saved_errno == EAGAIN || saved_errno == EWOULDBLOCK) { return 1; }
        RCLCPP_ERROR(logger_,
          "Error reading packet header from robot connection: %s",
          strerror(saved_errno));
        return 1;
      }
      if (cnt == 0) {
        continue;
      }

      if (prefix[0] == 0xFA && prefix[1] == 0xFB) {
        auto now = std::chrono::system_clock::now().time_since_epoch();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
        timestamp = rclcpp::Time(ns, RCL_SYSTEM_TIME);
        break;
      }

      prefix[0] = prefix[1];
      prefix[1] = prefix[2];
    }

    size = prefix[2] + 3;
    memcpy(packet, prefix, 3);

    cnt = 0;
    while (cnt != prefix[2]) {
      if (timeout_ms > 0) {
        if (!rclcpp::ok()) {
          return 2;
        }
        int ready = wait_for_data(fd, timeout_ms);
        if (ready == 0) { continue; }
        if (ready < 0) {
          if (errno == EINTR) { continue; }
          RCLCPP_ERROR(logger_, "select() error in Receive() body: %s", strerror(errno));
          return 1;
        }
      }

      int n = read(fd, &packet[3 + cnt], prefix[2] - cnt);
      if (n < 0) {
        if (errno == EINTR) { continue; }
        if ((errno == EAGAIN || errno == EWOULDBLOCK) && timeout_ms > 0) { continue; }
        RCLCPP_ERROR(logger_,
          "Error reading packet body from robot connection: %s", strerror(errno));
        return 1;
      }
      cnt += n;
    }
  } while (!Check());
  return 0;
}

int P2OSPacket::Build(unsigned char * data, unsigned char datasize)
{
  int16_t chksum;

  size = datasize + 5;

  /* header */
  packet[0] = 0xFA;
  packet[1] = 0xFB;

  if (size > 198) {
    RCLCPP_ERROR(logger_, "Packet to P2OS can't be larger than 200 bytes");
    return 1;
  }
  packet[2] = datasize + 2;

  memcpy(&packet[3], data, datasize);

  chksum = CalcChkSum();
  packet[3 + datasize] = chksum >> 8;
  packet[3 + datasize + 1] = chksum & 0xFF;

  if (!Check()) {
    RCLCPP_ERROR(logger_, "DAMN");
    return 1;
  }
  return 0;
}

int P2OSPacket::Send(int fd)
{
  int cnt = 0;

  while (cnt != size) {
    if ((cnt += write(fd, packet, size)) < 0) {
      RCLCPP_ERROR(logger_, "Send");
      return 1;
    }
  }
  return 0;
}
