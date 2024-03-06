/*
 * @Descripttion:
 * @Author: editorxu
 * @version:
 * @Date: 2024-02-29 14:15:44
 * @LastEditors: editorxu
 * @LastEditTime: 2024-03-06 10:01:07
 */
#include "master.h"

#include <sstream>
#include "yaml-cpp/yaml.h"

#include "ethercat_utils.h"
#include "slave_node.h"
#include "utils.h"

#include "log/ethercat_log.h"

namespace ether_backend {

ether_control::EthercatController g_controller;
ether_control::EtherConfig g_config;

//
void PreInitEther(const std::string &config_path) {
  YAML::Node cfg_node;
  try {
    cfg_node = YAML::LoadFile(config_path);
  } catch (...) {
    ETHER_ERROR("error loading file, yaml file error or not exist.");
    exit(EXIT_FAILURE);
  }

  if (auto ether_node = cfg_node["ethercat"]) {
    g_config.ifname = ether_node["ifname"].as<std::string>();
    g_config.cycle_time = ether_node["cycle_time"].as<uint64_t>();
    g_config.cycle_time = ether_node["enable_dc"].as<bool>();
    g_config.exclude_slave_list =
        ether_node["exclude_slave"].as<std::vector<int32_t>>();

    if (ether_node["thread_bind_cpus"].IsDefined()) g_config.thread_bind_cpus = ether_node["thread_bind_cpus"].as<std::vector<uint32_t>>();
    if (ether_node["thread_sched_policy"].IsDefined()) g_config.thread_sched_policy = ether_node["thread_sched_policy"].as<std::string>();

    if (auto slave_cfg_node = ether_node["slave_config"]) {
      for (const auto &config_itr : slave_cfg_node) {
        SlaveNodeConfig node_cfg;
        node_cfg.slave_no = config_itr["slave_no"].as<int32_t>();
        node_cfg.txpdo_addr = config_itr["txpdo_addr"].as<std::vector<uint16_t>>();
        node_cfg.rxpdo_addr = config_itr["rxpdo_addr"].as<std::vector<uint16_t>>();
        g_config.slave_config_list.push_back(node_cfg);
      }
    }
  }
  g_controller.EtherPreInitMasterNode(g_config);
}

void InitEther() {
  g_controller.EtherInitMasterNode(g_config);
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
    ETHER_INFO("rxpdo map, index: {}, addr: 0x{:4x}", index, addr);
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
    ETHER_INFO("txpdo map, index: {}, addr: 0x{:4x}", index, addr);
    WriteSDO(slave, 0x1C13, index, (uint16_t)addr);
  }
  u8val = (uint8_t)txpdo_map_.size();
  WriteSDO(slave, 0x1C13, 0, u8val);

  return 0;
}

//
void RegisterEtherNode(int32_t slave_no) {
  std::shared_ptr<SlaveNode> node =
      std::make_shared<SlaveNode>(g_controller.GetEtherManager(), slave_no);
  g_controller.EtherRegisterNode(slave_no, node, SlavePDOSetup);
}

//
void StartEther() {
  g_controller.EtherEnablePreSafeOP();
  g_controller.EtherConfigSlaveNode();
  g_controller.EtherEnableDC();
  g_controller.EtherEnableSafeOP();
  g_controller.EtherEnableOP();
  g_controller.EtherInitSlaveNodes();
  g_controller.EtherStart();
}
//
void StopEther() { g_controller.EtherStop(); }

//
std::string PrintEtherNodeList() {
  auto slave_list = g_controller.GetSlaveLists();
  return "[" + Display(slave_list) + "]";
}
//
std::string PrintEtherPDOMap(int32_t slave_no) {
  auto slave_node =
      std::dynamic_pointer_cast<SlaveNode>(g_controller.GetEtherNode(slave_no));

  std::stringstream ss;

  ss << "TXPDO:" << std::endl;
  ss << "name\tindex:sub\tbitlen\toffset\ttype" << std::endl;
  for (auto info : slave_node->txpdo_assign_vec_) {
    ss << fmt::format("{}\t0x{:4x}:0x{:2x}\t{}\t{}\t{}", info.name, info.index,
                      info.sub_index, info.bit_len, info.offset, info.data_type)
       << std::endl;
  }

  ss << "RXPDO:" << std::endl;
  ss << "name\tindex:sub\tbitlen\toffset\ttype" << std::endl;
  for (auto info : slave_node->rxpdo_assign_vec_) {
    ss << fmt::format("{}\t0x{:4x}:0x{:2x}\t{}\t{}\t{}", info.name, info.index,
                      info.sub_index, info.bit_len, info.offset, info.data_type)
       << std::endl;
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
}  // namespace ether_backend