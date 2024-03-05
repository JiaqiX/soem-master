#include "ethercat_utils.h"

#include "log/ethercat_log.h"

namespace ether_control {

template <typename T>
uint8_t WriteSDO(int32_t slave_no, uint16_t index, uint8_t subidx, T value) {
  uint8_t ret;
  ret = ec_SDOwrite(slave_no, index, subidx, FALSE, sizeof(value), &value,
                    EC_TIMEOUTSAFE);
  return ret;
}

template <typename T>
T ReadSDO(int32_t slave_no, uint16_t index, uint8_t subidx) {
  int32_t ret, l;
  T val;
  l = sizeof(val);
  ret = ec_SDOread(slave_no, index, subidx, FALSE, &l, &val, EC_TIMEOUTRXM);
  if (ret <= 0) {  // ret = Workcounter from last slave response
    ETHER_ERROR(
        "Failed to read from ret: {}, slave_no: {}, index:0x{:04x}, "
        "subidx:0x{:02x}",
        ret, slave_no, index, subidx);
  }
  return val;
}

template uint8_t WriteSDO<int8_t>(int32_t slave_no, uint16_t index,
                                  uint8_t subidx, int8_t value);
template uint8_t WriteSDO<int32_t>(int32_t slave_no, uint16_t index,
                                   uint8_t subidx, int32_t value);
template uint8_t WriteSDO<int16_t>(int32_t slave_no, uint16_t index,
                                   uint8_t subidx, int16_t value);
template uint8_t WriteSDO<uint8_t>(int32_t slave_no, uint16_t index,
                                   uint8_t subidx, uint8_t value);
template uint8_t WriteSDO<uint32_t>(int32_t slave_no, uint16_t index,
                                    uint8_t subidx, uint32_t value);
template uint8_t WriteSDO<uint16_t>(int32_t slave_no, uint16_t index,
                                    uint8_t subidx, uint16_t value);

template int8_t ReadSDO<int8_t>(int32_t slave_no, uint16_t index,
                                uint8_t subidx);
template int32_t ReadSDO<int32_t>(int32_t slave_no, uint16_t index,
                                  uint8_t subidx);
template int16_t ReadSDO<int16_t>(int32_t slave_no, uint16_t index,
                                  uint8_t subidx);
template uint8_t ReadSDO<uint8_t>(int32_t slave_no, uint16_t index,
                                  uint8_t subidx);
template uint32_t ReadSDO<uint32_t>(int32_t slave_no, uint16_t index,
                                    uint8_t subidx);
template uint16_t ReadSDO<uint16_t>(int32_t slave_no, uint16_t index,
                                    uint8_t subidx);

void DoubleToFixed(double f_input, int32_t *pValue, int32_t *pBase) {
  if (f_input < 1.0) {
    (*pBase) = 15;
    (*pValue) = (int32_t)(32768.0 * f_input);
  } else if (f_input < 2.0) {
    (*pBase) = 14;
    (*pValue) = (int32_t)(16384.0 * f_input);
  } else if (f_input < 4.0) {
    (*pBase) = 13;
    (*pValue) = (int32_t)(8192.0 * f_input);
  } else if (f_input < 8.0) {
    (*pBase) = 12;
    (*pValue) = (int32_t)(4096.0 * f_input);
  } else if (f_input < 16.0) {
    (*pBase) = 11;
    (*pValue) = (int32_t)(2048.0 * f_input);
  } else if (f_input < 32.0) {
    (*pBase) = 10;
    (*pValue) = (int32_t)(1024.0 * f_input);
  } else if (f_input < 64.0) {
    (*pBase) = 9;
    (*pValue) = (int32_t)(512.0 * f_input);
  } else if (f_input < 128.0) {
    (*pBase) = 8;
    (*pValue) = (int32_t)(256.0 * f_input);
  } else if (f_input < 256.0) {
    (*pBase) = 7;
    (*pValue) = (int32_t)(128.0 * f_input);
  } else if (f_input < 512.0) {
    (*pBase) = 6;
    (*pValue) = (int32_t)(64.0 * f_input);
  } else if (f_input < 1024.0) {
    (*pBase) = 5;
    (*pValue) = (int32_t)(32.0 * f_input);
  } else if (f_input < 2048.0) {
    (*pBase) = 4;
    (*pValue) = (int32_t)(16.0 * f_input);
  } else if (f_input < 4096.0) {
    (*pBase) = 3;
    (*pValue) = (int32_t)(8.0 * f_input);
  } else if (f_input < 81928.0) {
    (*pBase) = 2;
    (*pValue) = (int32_t)(4.0 * f_input);
  } else if (f_input < 16384.0) {
    (*pBase) = 1;
    (*pValue) = (int32_t)(2.0 * f_input);
  } else if (f_input < 32768.0) {
    (*pBase) = 0;
    (*pValue) = (int32_t)(1.0 * f_input);
  }
}
}  // namespace ether_control