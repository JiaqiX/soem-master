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

void EthercatController::EtherInitMasterNode(const EtherConfig &cfg) {
  ifname_ = cfg.ifname;
  enable_dc_ = cfg.enable_dc;
  cycle_time_ = cfg.cycle_time;
  exclude_slave_list_ = cfg.exclude_slave_list;
  manager_ = std::make_shared<EthercatManager>(ifname_, exclude_slave_list_,
                                               enable_dc_, cycle_time_);

  manager_->RegisterCallback(
      std::bind(&EthercatController::DataCallback, this));

  ETHER_INFO("parse ethercat cfg info: ifname: {}, cycle_time: {}, enable_dc: "
             "{}, exclude_slave_list: [{}]",
             ifname_, cycle_time_, enable_dc_, Display(exclude_slave_list_));
}

void EthercatController::EtherRegisterNode(int32_t slave_no,
                                           std::shared_ptr<EtherNode> node_ptr,
                                           PO2SOconfigFunc func) {
  ether_nodes_map_[slave_no] = node_ptr;
  manager_->SetSlaveConfig(slave_no, func);
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
void EthercatController::EtherEnablePreSafeOP() { manager_->EnablePreSafeOP(); }
void EthercatController::EtherConfigSlaveNode() { manager_->ConfigSlaveNode(); }
void EthercatController::EtherEnableDC() { manager_->EnableDC(); }
void EthercatController::EtherEnableSafeOP() { manager_->EnableSafeOP(); }
void EthercatController::EtherEnableOP() { manager_->EnableOP(); }
void EthercatController::EtherStart() {
  if (!stop_flag_.load())
    return;

  stop_flag_.store(false);

  cycle_thread_ =
      std::make_shared<std::thread>(&EthercatController::LoopWorker, this);
  handle_error_thread_ =
      std::make_shared<std::thread>(&EthercatController::HandleErrors, this);
}

void EthercatController::EtherStop() {
  if (stop_flag_.load())
    return;
  stop_flag_.store(true);

  manager_->SetStopFlag(true);

  if (cycle_thread_.get()) {
    cycle_thread_->join();
  }

  if (handle_error_thread_.get()) {
    handle_error_thread_->join();
  }

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
  if (ether_nodes_map_.find(slave_no) == ether_nodes_map_.end() ||
      stop_flag_.load())
    throw EthercatControllerError("can not get ethercat node.");
  return ether_nodes_map_[slave_no];
}

} // namespace ether_control