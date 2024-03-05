/*
 * @Descripttion:
 * @Author: editorxu
 * @version:
 * @Date: 2024-02-29 14:15:44
 * @LastEditors: editorxu
 * @LastEditTime: 2024-03-05 11:12:52
 */
#include "backend.h"

#include "yaml-cpp/yaml.h"
#include <sstream>

#include "ethercat_utils.h"
#include "slave_node.h"

#include "log/ethercat_log.h"

namespace ether_backend {

ether_control::EthercatController g_controller;
ether_control::EtherConfig g_config;

//
void InitEtherMaster(const std::string &config_path) {
  YAML::Node cfg_node;
  try {
    cfg_node = YAML::LoadFile(config_path);
  } catch (...) {
    ETHER_ERROR("error loading file, yaml file error or not exist.");
    exit(EXIT_FAILURE);
  }

  ether_control::EtherConfig cfg;
  if (auto ether_node = cfg_node["ethercat"]) {
    cfg.ifname = ether_node["ifname"].as<std::string>();
    cfg.cycle_time = ether_node["cycle_time"].as<uint64_t>();
    cfg.cycle_time = ether_node["enable_dc"].as<bool>();
    cfg.exclude_slave_list =
        ether_node["exclude_slave"].as<std::vector<int32_t>>();
  }

  g_controller.EtherInitMasterNode(cfg);
}

static int SlavePDOSetup(uint16_t slave) {
  uint8_t u8val = 0;
  uint16_t u16val = 0;

  /*
  example:
  RxPDO (1600h)       7000h 01h 8 LED
                      7000h 02h 8 OTestUint8
                      7000h 03h 16 control_word
                      7000h 03h 32 target_position_1
                      7000h 04h 32 target_velocity_1
                      7000h 05h 16 target_torque_1
                      ... ...
  */
  auto &rxpdo_map_ = g_controller.GetSlaveRxPDOMap(slave);

  u8val = (uint8_t)0;
  WriteSDO(slave, 0x1C12, 0, u8val);
  for (auto [index, addr] : rxpdo_map_) {
    WriteSDO(slave, 0x1C12, index, (uint16_t)addr);
  }

  u8val = (uint8_t)rxpdo_map_.size();
  WriteSDO(slave, 0x1C12, 0, u8val);
  /*
  example:
  TxPDO (1A00h)	      6000h 01h 8 Switch
                      6000h 02h 8 ITestUint8_2
                      6000h 03h 16 status_word
                      6000h 04h 32 actual_position_1
                      ... ...
  */
  auto &txpdo_map_ = g_controller.GetSlaveTxPDOMap(slave);

  u8val = (uint8_t)0;
  WriteSDO(slave, 0x1C13, 0, u8val);
  for (auto [index, addr] : txpdo_map_) {
    WriteSDO(slave, 0x1C13, index, (uint16_t)addr);
  }
  u8val = (uint8_t)txpdo_map_.size();
  WriteSDO(slave, 0x1C13, 0, u8val);

  return 0;
}

template <typename T>
  requires(requires(T t) {
             typename T::iterator;
             t.begin();
             t.end();
           })
std::string Display(const T &container) {
  std::stringstream ss;
  for (auto it = container.begin(); it != container.end(); ++it)
    ss << *it << ((it < container.end() - 1) ? ", " : "");
  return ss.str();
}
//
std::string PrintEtherNodeList() {

  auto slave_list = g_controller.GetSlaveLists();

  return Display(slave_list);
}
//
void RegisterEtherNode(int32_t slave_no) {
  std::shared_ptr<SlaveNode> node =
      std::make_shared<SlaveNode>(g_controller.GetEtherManager(), slave_no);
  node->InitSlaveNode();
  g_controller.EtherRegisterNode(slave_no, node, SlavePDOSetup);
}
//
void SetEtherNodeRxPDOMap(int32_t slave_no, uint32_t index, uint16_t addr) {
  g_controller.EtherSetSlaveNodeRxPDOMap(slave_no, index, addr);
}
//
void SetEtherNodeTxPDOMap(int32_t slave_no, uint32_t index, uint16_t addr) {
  g_controller.EtherSetSlaveNodeRxPDOMap(slave_no, index, addr);
}
//
void EnablePreSafeOP() { g_controller.EtherEnablePreSafeOP(); }
//
void ConfigSlaveNode() { g_controller.EtherConfigSlaveNode(); }
//
void EnableDC() { g_controller.EtherEnableDC(); }
//
void EnableSafeOP() { g_controller.EtherEnableSafeOP(); }
//
void EnableOP() { g_controller.EtherEnableOP(); }
//
void StartEther() { g_controller.EtherStart(); }
//
void StopEther() { g_controller.EtherStop(); }
//
std::string PrintEtherPDOMap(int32_t slave_no) {
  auto slave_node =
      std::dynamic_pointer_cast<SlaveNode>(g_controller.GetEtherNode(slave_no));

  std::stringstream ss;

  ss << "TXPDO:\n";
  ss << "index\tsub\tbitlen\toffset\ttype\n";
  for (auto [field_name, info] : slave_node->txpdo_assign_map_) {
    ss << info.index << "\t" << info.sub_index << "\t" << info.bit_len << "\t"
       << info.offset << "\t" << info.data_type << "\n";
  }

  ss << "\n";

  ss << "RXPDO:\n";
  ss << "index\tsub\tbitlen\toffset\ttype\n";
  for (auto [field_name, info] : slave_node->rxpdo_assign_map_) {
    ss << info.index << "\t" << info.sub_index << "\t" << info.bit_len << "\t"
       << info.offset << "\t" << info.data_type << "\n";
  }

  return ss.str();
}
//
void SetEtherNodeData(int32_t slave_no, const std::string &field_name,
                      std::string val) {
  auto slave_node =
      std::dynamic_pointer_cast<SlaveNode>(g_controller.GetEtherNode(slave_no));

  slave_node->SetOutput(field_name, val);
}
//
std::string GetEtherNodeData(int32_t slave_no, const std::string &field_name) {
  auto slave_node =
      std::dynamic_pointer_cast<SlaveNode>(g_controller.GetEtherNode(slave_no));
  return slave_node->GetInput(field_name);
}
} // namespace ether_backend