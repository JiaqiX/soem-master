#pragma once

#include <cstdint>
#include <stdexcept>

namespace ether_control {

enum class OPERATION_STATUS : uint8_t {
  NOT_READY,
  SWITCH_DISABLED,
  READY_SWITCH,
  SWITCHED_ON,
  OPERATION_ENABLED,
  QUICK_STOP,
  FAULT_REACTION,
  FAULT,
  UNKNOWN
};  // Status Word (6041h)

enum class OPERATION_CONTROL : uint8_t {
  CONTROLWORD_SWITCH_ON_BIT = 0,
  CONTROLWORD_ENABLE_VOLTAGE_BIT,
  CONTROLWORD_QUICK_STOP_BIT,
  CONTROLWORD_ENABLE_OPERATION_BIT,
  CONTROLWORD_FAULT_RESET_BIT = 7,
  CONTROLWORD_HALT_BIT = 8,
};  // Control Word (6040h)

enum class OPERATION_MODE : uint8_t {
  MODE_NONE = 0,
  MODE_PP = 1,    // Profile position mode
  MODE_VL = 2,    // Velocity mode
  MODE_PV = 3,    // Profile velocity mode
  MODE_TQ = 4,    // Torque profile mode
  MODE_HM = 6,    // Homing mode
  MODE_IP = 7,    // Interpolated position mode
  MODE_CSP = 8,   // Cyclic synchronous position mode
  MODE_CSV = 9,   // Cyclic synchronous velocity mode
  MODE_CST = 10,  // Cyclic synchronous torque mode
};                // Mode of operation(6060h)

}  // namespace ether_control