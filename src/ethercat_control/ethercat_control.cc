/*
 * @Descripttion:
 * @Author: editorxu
 * @version:
 * @Date: 2023-11-14 20:37:40
 * @LastEditors: editorxu
 * @LastEditTime: 2024-03-05 11:12:29
 */
#include "ethercat_control.h"
#include <sstream>

#include "ethercat_utils.h"

#include "log/ethercat_log.h"

namespace ether_control {

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

EthercatController::EthercatController() {}
EthercatController::~EthercatController() { EtherStop(); }

void EthercatController::EtherPreInitMasterNode(const EtherConfig &cfg) {
  manager_ = std::make_shared<EthercatManager>(cfg.ifname, cfg.exclude_slave_list,
                                                cfg.thread_bind_cpus, cfg.thread_sched_policy, cfg.enable_dc, cfg.cycle_time);
  manager_->RegisterCallback(
      std::bind(&EthercatController::DataCallback, this));
}

void EthercatController::EtherInitMasterNode(const EtherConfig &cfg) {

  manager_->InitMasterNode();

  // init pdo map by config
  for (auto slave_cfg: cfg.slave_config_list) {
    for (int index = 0; index < slave_cfg.txpdo_addr.size(); index++) {
      slave_txpdo_map_[slave_cfg.slave_no][index] = slave_cfg.txpdo_addr[index];
    }
    for (int index = 0; index < slave_cfg.rxpdo_addr.size(); index++) {
      slave_rxpdo_map_[slave_cfg.slave_no][index] = slave_cfg.rxpdo_addr[index];
    }
  }

  ETHER_INFO(
      "parse ethercat cfg info: ifname: {}, cycle_time: {}, enable_dc: "
      "{}, exclude_slave_list: [{}]",
      cfg.ifname, cfg.cycle_time, cfg.enable_dc, Display(cfg.exclude_slave_list));
}

void EthercatController::EtherRegisterNode(int32_t slave_no,
                                           std::shared_ptr<EtherNode> node_ptr,
                                           PO2SOconfigFunc func) {
  ETHER_INFO("register slave node: {}", slave_no);
  ether_nodes_map_[slave_no] = node_ptr;
  manager_->SetSlaveConfig(slave_no, func);
}


void EthercatController::EtherInitSlaveNodes() {
  for (auto [slave_no, node] : ether_nodes_map_) {
    node->InitSlaveNode();
  }
}

void EthercatController::EtherSetSlaveNodeTxPDOMap(int32_t slave_no,
                                                   uint32_t index,
                                                   uint16_t addr) {
  slave_txpdo_map_[slave_no][index] = addr;
}

void EthercatController::EtherSetSlaveNodeRxPDOMap(int32_t slave_no,
                                                   uint32_t index,
                                                   uint16_t addr) {
  slave_rxpdo_map_[slave_no][index] = addr;
}
void EthercatController::EtherEnablePreSafeOP() {
  if (!manager_->EnablePreSafeOP()) {
    throw EthercatControllerError("enable pre safe op failed.");
  }
}
void EthercatController::EtherConfigSlaveNode() {
  if (!manager_->ConfigSlaveNode()) {
    throw EthercatControllerError("config slave node failed.");
  }
}
void EthercatController::EtherEnableDC() {
  if (!manager_->EnableDC()) {
    throw EthercatControllerError("enable op failed.");
  }
}
void EthercatController::EtherEnableSafeOP() {
  if (!manager_->EnableSafeOP()) {
    throw EthercatControllerError("enbale safe op failed.");
  }
}
void EthercatController::EtherEnableOP() {
  if (!manager_->EnableOP()) {
    throw EthercatControllerError("enable op failed.");
  }
}
void EthercatController::EtherStart() {
  if (!stop_flag_.load()) {
    ETHER_WARN("ethercat worker loop task already start.");
    return;
  }
  ETHER_INFO("start ethercat worker loop task.");
  stop_flag_.store(false);

  cycle_thread_ =
      std::make_shared<std::thread>(&EthercatController::LoopWorker, this);

  handle_error_thread_ =
      std::make_shared<std::thread>(&EthercatController::HandleErrors, this);
}

void EthercatController::EtherStop() {
  if (stop_flag_.load())
    return;

  ETHER_INFO("stop ethercat worker loop task.");

  stop_flag_.store(true);

  manager_->SetStopFlag(true);

  if (cycle_thread_.get()) {
    cycle_thread_->join();
  }

  if (handle_error_thread_.get()) {
    handle_error_thread_->join();
  }

  manager_->ReleaseMasterNode();
  manager_.reset();
}

void EthercatController::LoopWorker() { manager_->LoopWorker(); }

void EthercatController::HandleErrors() { manager_->HandleErrors(); }

void EthercatController::DataCallback() {
  for (auto [node_no, node] : ether_nodes_map_) {
    node->GetRawInputs();
    node->WriteRawOutputs();
  }
}

void EthercatController::WaitUntil(
    std::chrono::time_point<std::chrono::high_resolution_clock> end_time) {
  return manager_->WaitUntil(end_time);
}

std::shared_ptr<EtherNode> EthercatController::GetEtherNode(int32_t slave_no) {
  if (ether_nodes_map_.find(slave_no) == ether_nodes_map_.end())
    throw EthercatControllerError("can not get ethercat node.");
  return ether_nodes_map_[slave_no];
}

}  // namespace ether_control