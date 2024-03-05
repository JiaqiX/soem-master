/*
 * @Descripttion:
 * @Author: editorxu
 * @version:
 * @Date: 2023-11-14 20:37:40
 * @LastEditors: editorxu
 * @LastEditTime: 2024-03-05 11:12:34
 */
#pragma once

#include <map>
#include <set>
#include <unordered_map>
#include <vector>

#include "ethercat_manager.h"
#include "ethercat_node.h"

using std::vector;

namespace ether_control {

typedef struct {
  std::string ifname;
  bool enable_dc = false;
  uint64_t cycle_time = 0;
  std::vector<int32_t> exclude_slave_list;
  bool enable_servo = false;
} EtherConfig;

class EthercatControllerError : public std::runtime_error {
 public:
  explicit EthercatControllerError(const std::string &what)
      : std::runtime_error(what) {}
};

class EthercatController {
 public:
  EthercatController();
  virtual ~EthercatController();

 private:
  void HandleErrors();
  void LoopWorker();

 public:
  void EtherInitMasterNode(const EtherConfig &cfg);
  void EtherRegisterNode(int32_t slave_no, std::shared_ptr<EtherNode> node_ptr,
                         PO2SOconfigFunc func);

  void EtherSetSlaveNodeTxPDOMap(int32_t slave_no, uint32_t index,
                                 uint16_t addr);
  void EtherSetSlaveNodeRxPDOMap(int32_t slave_no, uint32_t index,
                                 uint16_t addr);

  std::map<uint32_t, uint16_t> &GetSlaveTxPDOMap(int32_t slave_no) {
    return slave_txpdo_map_[slave_no];
  }
  std::map<uint32_t, uint16_t> &GetSlaveRxPDOMap(int32_t slave_no) {
    return slave_rxpdo_map_[slave_no];
  }

  void EtherEnablePreSafeOP();
  void EtherConfigSlaveNode();
  void EtherEnableDC();
  void EtherEnableSafeOP();
  void EtherEnableOP();
  void EtherStart();
  void EtherStop();

  std::vector<int32_t> GetSlaveLists() { return manager_->GetSlaveLists(); }
  uint16_t GetEtherNodeNum() { return ether_nodes_map_.size(); }

 public:
  std::shared_ptr<EthercatManager> GetEtherManager() { return manager_; }
  std::shared_ptr<EtherNode> GetEtherNode(int32_t slave_no);

  // data callback function
  void DataCallback();
  void WaitUntil(
      std::chrono::time_point<std::chrono::high_resolution_clock> end_time);

 private:
  std::atomic_bool stop_flag_ = true;
  std::string ifname_;
  uint64_t cycle_time_;
  bool enable_dc_;
  std::vector<int32_t> exclude_slave_list_;

  std::shared_ptr<EthercatManager> manager_;
  std::unordered_map<int32_t, std::shared_ptr<EtherNode>> ether_nodes_map_;

  // cycle date process
  std::shared_ptr<std::thread> cycle_thread_;
  std::shared_ptr<std::thread> handle_error_thread_;

 private:
  std::map<int32_t, std::map<uint32_t, uint16_t>> slave_txpdo_map_;
  std::map<int32_t, std::map<uint32_t, uint16_t>> slave_rxpdo_map_;

};  // class EthercatController
}  // namespace ether_control