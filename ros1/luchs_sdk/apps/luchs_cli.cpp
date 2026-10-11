// Copyright 2026 TerraAgent Robotics. Chassis: RJ Tech Limited.
// SPDX-License-Identifier: Apache-2.0
//
// luchs_cli - command-line tool for Luchs chassis (Luchs A and Luchs B, no ROS needed).
//
//   luchs_cli dump    [-i can0] [--build b] [--protocol P]     print every frame, decoded
//   luchs_cli monitor [-i can0] [--build b] [--protocol P] [--raw]   live state table (2 Hz)
//   luchs_cli encode  [--build 4ws|b] [--protocol P] [--vx V] [--vy V] [--wz W] [--model M]
//                                                  print the frames one cycle would send
//   luchs_cli drive   -i can0 --build b --vx 0.2 [--wz 0] [--seconds 2] --yes [--sim]
//                                                  drive with watchdog, then park
//   luchs_cli version
//
// --protocol: a (Luchs A v1.1), lci (LCI v1.0), legacy_b (Luchs B VCU upper interface).
// Default: lci for --build b, a otherwise.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <thread>

#include "luchs_sdk/chassis.hpp"
#include "luchs_sdk/sim_bus.hpp"

using namespace luchs_sdk;

namespace
{
std::atomic<bool> g_stop{false};
void on_signal(int) {g_stop = true;}

struct Args
{
  std::string cmd;
  std::map<std::string, std::string> kv;
  bool has(const std::string & k) const {return kv.count(k) > 0;}
  std::string get(const std::string & k, const std::string & d) const
  {
    auto it = kv.find(k);
    return it == kv.end() ? d : it->second;
  }
  double num(const std::string & k, double d) const
  {
    return has(k) ? std::atof(kv.at(k).c_str()) : d;
  }
};

Args parse(int argc, char ** argv)
{
  Args a;
  if (argc > 1) {a.cmd = argv[1];}
  for (int i = 2; i < argc; ++i) {
    std::string k = argv[i];
    if (k == "-i") {k = "--interface";}
    if (k.rfind("--", 0) != 0) {continue;}
    k = k.substr(2);
    if (i + 1 < argc && std::strncmp(argv[i + 1], "--", 2) != 0 && argv[i + 1][0] != '-') {
      a.kv[k] = argv[++i];
    } else if (i + 1 < argc && argv[i + 1][0] == '-' && std::isdigit(argv[i + 1][1])) {
      a.kv[k] = argv[++i];       // negative number
    } else {
      a.kv[k] = "1";
    }
  }
  return a;
}

bool parse_build(const std::string & s, Build & b)
{
  if (s == "df") {b = Build::kSkidSteer; return true;}
  if (s == "fa") {b = Build::kFrontAckermann; return true;}
  if (s == "da") {b = Build::kDualAckermann; return true;}
  if (s == "4ws") {b = Build::kFourWheelSteer; return true;}
  if (s == "b") {b = Build::kLuchsB; return true;}
  return false;
}

bool parse_protocol(const Args & a, Build b, Protocol & p)
{
  const std::string s = a.get("protocol", b == Build::kLuchsB ? "lci" : "a");
  if (s == "a") {p = Protocol::kLegacyA; return true;}
  if (s == "lci") {p = Protocol::kLci; return true;}
  if (s == "legacy_b") {p = Protocol::kLegacyB; return true;}
  return false;
}

/// Options for --build / --protocol, or false after printing an error.
bool options_from(const Args & a, ChassisOptions & o)
{
  Build b = Build::kFourWheelSteer;
  Protocol p = Protocol::kLegacyA;
  if (!parse_build(a.get("build", "4ws"), b) || !parse_protocol(a, b, p)) {
    std::fprintf(stderr, "error: --build must be df, fa, da, 4ws or b; --protocol a, lci or "
      "legacy_b\n");
    return false;
  }
  o = ChassisOptions::for_build(b, p);
  o.interface = a.get("interface", "can0");
  o.lci.reference_x = -o.geometry.wheelbase / 2.0;
  return true;
}

bool parse_model(const std::string & s, MotionModel & m)
{
  if (s == "diff") {m = MotionModel::kDifferential; return true;}
  if (s == "ackermann") {m = MotionModel::kFrontAckermann; return true;}
  if (s == "dual_ackermann") {m = MotionModel::kDualAckermann; return true;}
  if (s == "4ws") {m = MotionModel::kFourWheelSteer; return true;}
  return false;
}

int usage()
{
  std::puts(
    "luchs_cli - Luchs chassis command-line tool\n\n"
    "  luchs_cli dump    [-i can0] [--build B] [--protocol P]\n"
    "  luchs_cli monitor [-i can0] [--build B] [--protocol P] [--raw]\n"
    "  luchs_cli encode  [--build B] [--protocol P] [--vx m/s] [--vy m/s] [--wz rad/s] "
    "[--model diff|ackermann|dual_ackermann|4ws]\n"
    "  luchs_cli drive   -i can0 --build B [--protocol P] --vx 0.2 [--vy 0] [--wz 0] "
    "[--seconds 2] --yes [--sim]\n"
    "  luchs_cli version\n\n"
    "  B: df | fa | da | 4ws (Luchs A builds) | b (Luchs B)\n"
    "  P: a (Luchs A v1.1) | lci (LCI v1.0, default for b) | legacy_b (Luchs B VCU V2.6)\n\n"
    "Bring the interface up first:  sudo ip link set can0 up type can bitrate 1000000");
  return 2;
}

void print_state(const ChassisState & s, bool raw)
{
  auto age = [](const std::optional<Clock::time_point> & t) {
      if (!t) {return std::string("   --  ");}
      char b[16];
      std::snprintf(b, sizeof(b), "%5.0fms",
        std::chrono::duration<double, std::milli>(Clock::now() - *t).count());
      return std::string(b);
    };
  const char * src = s.motion.source == ControlSource::kRemoteControl ? "REMOTE" :
    s.motion.source == ControlSource::kHost ? "HOST" : "?";
  std::printf("\033[2J\033[H");
  std::printf("Luchs chassis  frames ok %llu  unknown %llu\n\n",
    static_cast<unsigned long long>(s.frames_received),
    static_cast<unsigned long long>(s.frames_unknown));
  std::printf("motion   %s  vx %+6.3f m/s  vy %+6.3f m/s  wz %+6.3f rad/s  model 0x%02X  "
    "control 0x%02X (%s)\n", age(s.motion_stamp).c_str(), s.motion.twist.vx, s.motion.twist.vy,
    s.motion.twist.wz, s.motion.model_raw, s.motion.control_raw, src);
  std::printf("steer    %s  gear %u  front %+6.3f rad  rear %+6.3f rad\n",
    age(s.steer_stamp).c_str(), s.steer.gear_raw, s.steer.front_rad, s.steer.rear_rad);
  std::printf("wheels   %s  steer LF %+6.3f RF %+6.3f LR %+6.3f RR %+6.3f rad\n",
    age(s.wheel_steer_stamp).c_str(), s.wheel_steer_rad[0], s.wheel_steer_rad[1],
    s.wheel_steer_rad[2], s.wheel_steer_rad[3]);
  std::printf("speeds   %s  LF %+7.3f RF %+7.3f LR %+7.3f RR %+7.3f", age(s.wheel_speed_stamp).c_str(),
    s.wheel_speed[0], s.wheel_speed[1], s.wheel_speed[2], s.wheel_speed[3]);
  if (raw) {
    std::printf("   raw %d %d %d %d", s.wheel_speed_raw[0], s.wheel_speed_raw[1],
      s.wheel_speed_raw[2], s.wheel_speed_raw[3]);
  }
  std::printf("\nsafety   %s  ultrasonic trig %u  bumper %u  e-stop %u  fault 0x%02X (%s)\n",
    age(s.safety_stamp).c_str(), s.safety.ultrasonic_trigger_raw, s.safety.bumper_raw,
    s.safety.estop_raw, s.safety.fault_code, fault_code_text(s.safety.fault_code).c_str());
  std::printf("         ultrasonic F %.2f R %.2f L %.2f Rt %.2f m", s.safety.ultrasonic_m[0],
    s.safety.ultrasonic_m[1], s.safety.ultrasonic_m[2], s.safety.ultrasonic_m[3]);
  if (raw) {
    std::printf("   raw %u %u %u %u", s.safety.ultrasonic_raw[0], s.safety.ultrasonic_raw[1],
      s.safety.ultrasonic_raw[2], s.safety.ultrasonic_raw[3]);
  }
  std::printf("\nbattery  %s  SoC %.1f %%  %.1f V  %.1f C  charging %.0f %%",
    age(s.battery_stamp).c_str(), s.battery.soc_percent, s.battery.voltage_v,
    s.battery.temperature_c, s.battery.charging_percent);
  if (raw) {
    std::printf("   raw soc %u temp %u chg %u volt %u", s.battery.raw[0], s.battery.raw[1],
      s.battery.raw[2], s.battery.raw[3]);
  }
  std::printf("\nmotors   %s  fault high 0x%08X low 0x%08X\n", age(s.motor_fault_stamp).c_str(),
    s.motor_fault_high, s.motor_fault_low);
  std::printf("lamps    %s  turn %u reverse %u fault %u light/rain 0x%02X\n",
    age(s.lamps_stamp).c_str(), s.lamps.turn_raw, s.lamps.reverse_raw, s.lamps.fault_raw,
    s.lamps.light_rain_raw);
  if (s.status_stamp || s.limits_stamp || s.version_stamp) {
    static const char * kMode[] = {"maintenance", "remote control", "automatic"};
    std::printf("\nstate    %s  mode %s  fault level %u  e-stop source: %s\n",
      age(s.status_stamp).c_str(), s.status.operating_mode < 3 ? kMode[s.status.operating_mode] : "?",
      s.status.fault_level, estop_source_text(s.status.estop_source_bits).c_str());
    std::printf("         RC online %d enabled %d link %u  4WD %d  park brake %d  clipped %d  "
      "speed cap %.2f m/s  alive %u  checksum errors %llu\n", s.status.rc_online,
      s.status.rc_enabled, s.status.rc_link, s.status.four_wd, s.status.park_brake,
      s.status.cmd_clipped, s.status.active_speed_limit, s.status.alive_counter,
      static_cast<unsigned long long>(s.status.checksum_errors));
    std::printf("limits   %s  fwd %.2f rev %.2f m/s  steer %.3f rad  reason: %s  clipped speed %d "
      "steer %d accel %d\n", age(s.limits_stamp).c_str(), s.limits.max_vx_fwd, s.limits.max_vx_rev,
      s.limits.max_steer_rad, limit_reason_text(s.limits.reason_bits).c_str(),
      s.limits.speed_clipped, s.limits.steer_clipped, s.limits.accel_clipped);
    std::printf("imu      %s  rate %+.3f %+.3f %+.3f rad/s  accel %+.2f %+.2f %+.2f m/s2  "
      "roll %+.3f pitch %+.3f rad\n", age(s.imu_rate_stamp).c_str(), s.imu.rate[0], s.imu.rate[1],
      s.imu.rate[2], s.imu.accel[0], s.imu.accel[1], s.imu.accel[2], s.imu.roll, s.imu.pitch);
    std::printf("brake    %s  %.2f MPa  EPB %u  EPS front %u rear %u  brake %u\n",
      age(s.brake_stamp).c_str(), s.brake.brake_pressure_mpa, s.brake.epb_state,
      s.brake.eps_front, s.brake.eps_rear, s.brake.brake_state);
    std::printf("temps    %s  motors %.0f %.0f %.0f %.0f C  controllers %.0f %.0f %.0f %.0f C\n",
      age(s.temperature_stamp).c_str(), s.temperature.motor[0], s.temperature.motor[1],
      s.temperature.motor[2], s.temperature.motor[3], s.temperature.controller[0],
      s.temperature.controller[1], s.temperature.controller[2], s.temperature.controller[3]);
    std::printf("version  %s  LCI %u.%u  %s  firmware %u.%u.%u\n", age(s.version_stamp).c_str(),
      s.version.lci_major, s.version.lci_minor, model_id_text(s.version.model_id).c_str(),
      s.version.fw_major, s.version.fw_minor, s.version.fw_patch);
  }
  std::fflush(stdout);
}

bool decode_any(const ChassisOptions & o, const CanFrame & f, ChassisState & st)
{
  switch (o.protocol_type) {
    case Protocol::kLegacyA: return legacy_a::decode(f, o.protocol, st);
    case Protocol::kLci: return lci::decode(f, o.lci, st);
    case Protocol::kLegacyB: return legacy_b::decode(f, o.legacy_b, st);
  }
  return false;
}

std::string describe_any(const ChassisOptions & o, const CanFrame & f)
{
  switch (o.protocol_type) {
    case Protocol::kLegacyA: return legacy_a::describe(f);
    case Protocol::kLci: return lci::describe(f);
    case Protocol::kLegacyB: return legacy_b::describe(f, o.legacy_b);
  }
  return {};
}

int cmd_listen(const Args & a, bool monitor)
{
  ChassisOptions o;
  if (!options_from(a, o)) {return 2;}
  SocketCan can;
  if (!can.open(o.interface)) {
    std::fprintf(stderr, "error: %s\n", can.last_error().c_str());
    return 1;
  }
  ChassisState st;
  auto last = Clock::now();
  while (!g_stop) {
    auto f = can.read(50);
    if (f) {
      decode_any(o, *f, st);
      if (!monitor) {std::printf("%s\n", describe_any(o, *f).c_str());}
    }
    if (monitor && Clock::now() - last > std::chrono::milliseconds(500)) {
      print_state(st, a.has("raw"));
      last = Clock::now();
    }
  }
  return 0;
}

int cmd_encode(const Args & a)
{
  ChassisOptions o;
  if (!options_from(a, o)) {return 2;}
  const Build b = o.geometry.build;
  const Geometry & g = o.geometry;
  MotionModel m = default_model(b);
  if (a.has("model") && (!parse_model(a.get("model", ""), m) || !model_supported(b, m))) {
    std::fprintf(stderr, "error: model not supported by this build\n");
    return 1;
  }
  Twist2D t{a.num("vx", 0.0), a.num("vy", 0.0), a.num("wz", 0.0)};
  const SteerSolution s = solve(t, m, g);
  MotionCommand c;
  c.twist = s.twist;
  c.model = s.model;
  std::printf("%s, %s\n", build_name(b), protocol_name(o.protocol_type));
  std::printf("request  vx %+.3f vy %+.3f wz %+.3f  ->  commanded vx %+.3f vy %+.3f wz %+.3f%s\n",
    t.vx, t.vy, t.wz, s.twist.vx, s.twist.vy, s.twist.wz, s.limited ? "  (limited)" : "");
  std::printf("model 0x%02X  front %+.3f rad  rear %+.3f rad  wheels LF %+.3f RF %+.3f LR %+.3f "
    "RR %+.3f rad\n", static_cast<unsigned>(s.model), s.front_rad, s.rear_rad, s.wheel_rad[0],
    s.wheel_rad[1], s.wheel_rad[2], s.wheel_rad[3]);
  std::vector<CanFrame> frames;
  switch (o.protocol_type) {
    case Protocol::kLegacyA:
      frames = legacy_a::encode_cycle(c, s.wheel_rad, s.front_rad, s.rear_rad, LampCommand{},
          o.protocol);
      break;
    case Protocol::kLci:
      frames = lci::encode_cycle(c, s.front_rad, s.rear_rad, lci::SteerSource::kYawRate, 1,
          HostRequest{}, o.lci, LampCommand{});
      break;
    case Protocol::kLegacyB: {
        legacy_b::SpeedController sc;
        const auto p = sc.step(c.twist.vx, 0.0, 0.0, o.legacy_b);
        frames = {legacy_b::encode_command(p, s.front_rad, std::abs(c.twist.vx) * 1.1 + 0.3,
            c.twist.vx < 0.0 ? Gear::kReverse : Gear::kDrive, HostRequest{}, LampCommand{}, 1,
            o.legacy_b)};
        std::printf("pedals from rest: throttle %.0f %%  brake %.0f %%\n", p.throttle, p.brake);
        break;
      }
  }
  for (const auto & f : frames) {std::printf("  %s\n", describe_any(o, f).c_str());}
  return 0;
}

int cmd_drive(const Args & a)
{
  if (!a.has("yes")) {
    std::fprintf(stderr, "drive moves the vehicle. Lift the wheels or clear the area, keep the "
      "emergency stop in reach, then add --yes.\n");
    return 1;
  }
  ChassisOptions opt;
  if (!options_from(a, opt)) {return 2;}
  const Build b = opt.geometry.build;
  opt.geometry.max_vx = 0.5;     // conservative limits for a bench test
  opt.geometry.max_vy = b == Build::kFourWheelSteer ? 0.3 : 0.0;
  opt.geometry.max_wz = 0.8;
  MotionModel m = default_model(b);
  if (a.has("model") && parse_model(a.get("model", ""), m)) {opt.model = m;}
  if (a.has("sim")) {std::printf("simulator (no CAN)\n");}
  Chassis ch(opt, a.has("sim") ? make_simulator(opt) : nullptr);
  if (!ch.start()) {
    std::fprintf(stderr, "error: %s\n", ch.last_error().c_str());
    return 1;
  }
  const Twist2D t{a.num("vx", 0.0), a.num("vy", 0.0), a.num("wz", 0.0)};
  const double secs = std::min(a.num("seconds", 2.0), 10.0);
  const auto end = Clock::now() + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(secs));
  while (!g_stop && Clock::now() < end) {
    ch.set_twist(t);
    const auto st = ch.status();
    std::printf("\rcmd vx %+.2f vy %+.2f wz %+.2f  feedback %s  gear %u%s   ",
      st.last.twist.vx, st.last.twist.vy, st.last.twist.wz, st.feedback_ok ? "ok" : "MISSING",
      static_cast<unsigned>(st.gear), st.limited ? "  limited" : "");
    std::fflush(stdout);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ch.halt();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ch.stop();
  std::printf("\nstopped, gear P\n");
  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  const Args a = parse(argc, argv);
  if (a.cmd == "dump") {return cmd_listen(a, false);}
  if (a.cmd == "monitor") {return cmd_listen(a, true);}
  if (a.cmd == "encode") {return cmd_encode(a);}
  if (a.cmd == "drive") {return cmd_drive(a);}
  if (a.cmd == "version") {
    std::printf("luchs_sdk %s  (protocols: Luchs A v1.1, LCI v%u.%u draft r3, Luchs B VCU V2.6)\n",
      kSdkVersion, lci::kLciMajor, lci::kLciMinor);
    return 0;
  }
  return usage();
}
