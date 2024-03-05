/*
 * @Descripttion:
 * @Author: editorxu
 * @version:
 * @Date: 2023-11-14 20:37:40
 * @LastEditors: editorxu
 * @LastEditTime: 2024-03-05 11:12:21
 */
#pragma once

#include <stdint.h>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ethercat_sem.h"

namespace ether_control {

typedef int (*PO2SOconfigFunc)(uint16_t);
using CallbackFunc = std::function<void(void)>;

class EthercatError : public std::runtime_error {
 public:
  explicit EthercatError(const std::string &what) : std::runtime_error(what) {}
};

class EthercatManager {
 public:
  EthercatManager(const std::string &ifname,
                  const std::vector<int32_t> &exclude_slave_list,
                  const std::vector<uint32_t> &thread_bind_cpus,
                  const std::string &thread_sched_policy,
                  bool dc_enable = false,
                  uint64_t period = 1000000 /* default 1ms */
  );
  virtual ~EthercatManager();

  int GetSlaveNum() const;
  void GetStatus(int32_t slave_no, std::string &name, uint16_t &configadr,
                 uint16_t &aliasadr, uint32_t &eep_man, uint32_t &eep_id,
                 uint32_t &eep_rev, uint8_t &hasdc, uint16_t &obits_size,
                 uint16_t &ibits_size) const;
  int64_t CalcDcPiSync(int64_t _refTime, int64_t _cycleTime,
                       int64_t _shiftTime);

 public:
  bool InitMasterNode();
  bool EnablePreSafeOP();
  bool ConfigSlaveNode();
  bool EnableDC();
  bool EnableSafeOP();
  bool EnableOP();
  void ReleaseMasterNode();

 public:
  std::vector<int32_t> GetSlaveLists() { return slave_list_; }

 public:
  void HandleErrors();
  void LoopWorker();

 public:
  // elmo driver specific config
  void SlaveConfig();
  void SetSlaveConfig(int slave_idx, PO2SOconfigFunc func);

  // Read Inputs
  void ReadInputs(int slave_no, void *inputs, size_t size);
  // Write Outputs
  void WriteOutputs(int slave_no, const void *outputs, size_t size);
  // Read Outputs
  void ReadOutputs(int slave_no, void *outputs, size_t size);

  // Register callback function
  void RegisterCallback(CallbackFunc func) { callback_ = func; }

  void SetStopFlag(bool state) { stop_flag_ = state; }

  void Notify() { sem_.Notify(); }

  void WaitUntil(
      std::chrono::time_point<std::chrono::high_resolution_clock> end_time) {
    sem_.WaitUntil(end_time);
  }

 public:
  uint8_t iomap_[4096] = {0};

 protected:
  const std::string ifname_;
  int slave_num_ = 0;

  std::atomic_bool stop_flag_ = false;
  // cycle time
  uint64_t period_ = 1000000;  // 1 ms
  // dc
  bool dc_enable_ = false;
  // exclude first slave
  std::unordered_set<int> exclude_slave_set_;
  // thread bind cpus
  std::string thread_sched_policy_;
  // thread sched policy
  std::vector<uint32_t> cpu_set_;

 private:
  // workcount
  int sent = 0;
  int wkc = 0;
  int expected_wkc = 0;
  // slave list
  std::vector<int32_t> slave_list_;

  // slave pdo map
  std::unordered_map<int32_t, PO2SOconfigFunc> pdo_map_;

  // callback
  CallbackFunc callback_;

  // Sem
  Sem sem_;
};

}  // namespace ether_control