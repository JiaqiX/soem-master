#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
///
#include <ethercattype.h>
#include <nicdrv.h>
///
#include <ethercatmain.h>
///
#include <ethercatbase.h>
#include <ethercatcoe.h>
#include <ethercatconfig.h>
#include <ethercatdc.h>
#include <ethercatfoe.h>
#include <ethercatprint.h>

namespace ether_control {
template <typename T>
uint8_t WriteSDO(int slave_no, uint16_t index, uint8_t subidx, T value);
template <typename T>
T ReadSDO(int slave_no, uint16_t index, uint8_t subidx);

void DoubleToFixed(double f_input, int32_t *pValue, int32_t *pBase);

}  // namespace ether_control