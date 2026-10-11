// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0

#include "luchs_sdk/socketcan.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace luchs_sdk
{

SocketCan::~SocketCan() {close();}

bool SocketCan::open(const std::string & interface, const std::vector<uint32_t> & filter_ids)
{
  close();
  ifname_ = interface;
  if (interface.empty() || interface.size() >= IFNAMSIZ) {
    error_ = "invalid CAN interface name '" + interface + "'";
    return false;
  }
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    error_ = std::string("socket(PF_CAN): ") + std::strerror(errno);
    return false;
  }
  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    error_ = "CAN interface '" + interface + "' not found (" + std::strerror(errno) +
      "). Bring it up first: sudo ip link set " + interface + " up type can bitrate 1000000";
    close();
    return false;
  }
  if (!filter_ids.empty()) {
    std::vector<can_filter> f(filter_ids.size());
    for (std::size_t i = 0; i < filter_ids.size(); ++i) {
      f[i].can_id = filter_ids[i];
      f[i].can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
    }
    ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, f.data(),
      static_cast<socklen_t>(f.size() * sizeof(can_filter)));
  }
  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    error_ = std::string("bind: ") + std::strerror(errno);
    close();
    return false;
  }
  const int flags = ::fcntl(fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    error_ = std::string("fcntl: ") + std::strerror(errno);
    close();
    return false;
  }
  error_.clear();
  return true;
}

void SocketCan::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SocketCan::write(const CanFrame & frame)
{
  if (fd_ < 0) {return false;}
  struct can_frame f {};
  f.can_id = frame.id & CAN_SFF_MASK;
  f.can_dlc = frame.dlc > 8 ? 8 : frame.dlc;
  std::memcpy(f.data, frame.data.data(), 8);
  const auto n = ::write(fd_, &f, sizeof(f));
  if (n != static_cast<ssize_t>(sizeof(f))) {
    ++tx_errors_;
    error_ = std::string("write: ") + (n < 0 ? std::strerror(errno) : "short write");
    return false;
  }
  return true;
}

std::optional<CanFrame> SocketCan::read(int timeout_ms)
{
  if (fd_ < 0) {return std::nullopt;}
  struct pollfd p {fd_, POLLIN, 0};
  if (::poll(&p, 1, timeout_ms) <= 0 || !(p.revents & POLLIN)) {return std::nullopt;}
  struct can_frame f {};
  const auto n = ::read(fd_, &f, sizeof(f));
  if (n != static_cast<ssize_t>(sizeof(f))) {return std::nullopt;}
  if (f.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG)) {return std::nullopt;}
  CanFrame out;
  out.id = f.can_id & CAN_SFF_MASK;
  out.dlc = f.can_dlc;
  std::memcpy(out.data.data(), f.data, 8);
  return out;
}

}  // namespace luchs_sdk
