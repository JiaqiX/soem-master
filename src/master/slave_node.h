/*
 * @Descripttion:
 * @Author: editorxu
 * @version:
 * @Date: 2024-02-28 10:22:41
 * @LastEditors: editorxu
 * @LastEditTime: 2024-03-05 10:20:31
 */
#pragma once

#include <any>
#include <iostream>
#include <map>

#include "ethercat.h"
#include "ethercat_com.h"
#include "ethercat_node.h"

namespace ether_backend {

using namespace ether_control;

typedef struct data_info {
  std::string name;
  uint16_t index;
  uint8_t sub_index;
  uint8_t bit_len;
  uint32_t offset;
  std::string data_type;
  data_info() {}
  data_info(const data_info &other) {
    name = other.name;
    index = other.index;
    sub_index = other.sub_index;
    bit_len = other.bit_len;
    offset = other.offset;
    data_type = other.data_type;
  }

} DataInfo;

class SlaveNodeError : public std::runtime_error {
 public:
  SlaveNodeError(const std::string &what, int slave_no)
      : std::runtime_error(what + " slave no: " + std::to_string(slave_no)) {}
  SlaveNodeError(const std::string &what) : std::runtime_error(what) {}
};

class SlaveNode : public EtherNode {
 public:
  SlaveNode(std::weak_ptr<EthercatManager> manager, int32_t slave_no);
  virtual ~SlaveNode();

  virtual void InitSlaveNode() override;

  virtual void GetRawInputs() override;
  virtual void GetRawOutputs() override;
  virtual void WriteRawOutputs() override;

  // Product Code
  uint32_t GetProductCode();
  bool CheckProductCode(uint32_t code);

 private:
  // field read
  bool GetInputBool(int32_t offset);
  int8_t GetInputInt8(int32_t offset);
  uint8_t GetInputUint8(int32_t offset);
  int16_t GetInputInt16(int32_t offset);
  uint16_t GetInputUint16(int32_t offset);
  int32_t GetInputInt32(int32_t offset);
  uint32_t GetInputUint32(int32_t offset);
  int64_t GetInputInt64(int32_t offset);
  uint64_t GetInputUint64(int32_t offset);

  // field write
  void SetOutputBool(int32_t offset, bool val);
  void SetOutputInt8(int32_t offset, int8_t val);
  void SetOutputUint8(int32_t offset, uint8_t val);
  void SetOutputInt16(int32_t offset, int16_t val);
  void SetOutputUint16(int32_t offset, uint16_t val);
  void SetOutputInt32(int32_t offset, int32_t val);
  void SetOutputUint32(int32_t offset, uint32_t val);
  void SetOutputInt64(int32_t offset, int64_t val);
  void SetOutputUint64(int32_t offset, uint64_t val);

 public:
  std::string GetInput(const std::string &field_name);
  void SetOutput(const std::string &field_name, std::string strVal);

 private:
  std::string dtype2string(uint16_t dtype);
  int InitPDOMap();
  /** Read PDO assign structure */
  int PDOAssign(uint8_t tSM, uint16_t PDOassign, int mapoffset, int bitoffset);

  ec_ODlistt ODlist;
  ec_OElistt OElist;

 public:
  std::map<std::string, DataInfo> txpdo_assign_map_;
  std::map<std::string, DataInfo> rxpdo_assign_map_;

  std::vector<DataInfo> txpdo_assign_vec_;
  std::vector<DataInfo> rxpdo_assign_vec_;

 private:
  std::mutex mtx_input_;
  std::mutex mtx_output_;

  std::vector<uint8_t> inputs;
  std::vector<uint8_t> outputs;
};
}  // namespace ether_backend