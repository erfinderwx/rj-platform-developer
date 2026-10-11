// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Minimal Linux SocketCAN transport (raw socket, non-blocking, optional receive filter).

#ifndef LUCHS_SDK__SOCKETCAN_HPP_
#define LUCHS_SDK__SOCKETCAN_HPP_

#include <optional>
#include <string>
#include <vector>

#include "luchs_sdk/types.hpp"

namespace luchs_sdk
{

/// Abstract CAN transport, so the chassis client can run on SocketCAN or on a test double.
class Transport
{
public:
  virtual ~Transport() = default;
  virtual bool write(const CanFrame & frame) = 0;
  /// Waits up to `timeout_ms` (0 = poll) for one frame.
  virtual std::optional<CanFrame> read(int timeout_ms) = 0;
  virtual bool is_open() const = 0;
  virtual const std::string & last_error() const = 0;
};

class SocketCan : public Transport
{
public:
  SocketCan() = default;
  ~SocketCan() override;
  SocketCan(const SocketCan &) = delete;
  SocketCan & operator=(const SocketCan &) = delete;

  /// Opens and binds `interface` (e.g. "can0", "vcan0"). Returns false and sets last_error().
  /// With `filter_ids` non-empty, only those identifiers are delivered by the kernel.
  bool open(const std::string & interface, const std::vector<uint32_t> & filter_ids = {});
  void close();
  bool is_open() const override {return fd_ >= 0;}

  bool write(const CanFrame & frame) override;
  std::optional<CanFrame> read(int timeout_ms) override;

  const std::string & last_error() const override {return error_;}
  const std::string & interface() const {return ifname_;}
  uint64_t tx_errors() const {return tx_errors_;}

private:
  int fd_{-1};
  std::string ifname_;
  std::string error_;
  uint64_t tx_errors_{0};
};

}  // namespace luchs_sdk

#endif  // LUCHS_SDK__SOCKETCAN_HPP_
