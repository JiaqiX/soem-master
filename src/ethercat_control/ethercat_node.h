/*
 * @Descripttion:
 * @Author: editorxu
 * @version:
 * @Date: 2023-11-14 20:37:40
 * @LastEditors: editorxu
 * @LastEditTime: 2024-03-05 11:18:13
 */
#pragma once

#include <cstdint>
#include <map>
#include <stdexcept>
#include "ethercat_com.h"
#include "ethercat_manager.h"

#include "log/ethercat_log.h"

namespace ether_control {

class EtherNodeError : public std::runtime_error {
 public:
  explicit EtherNodeError(const std::string &what) : std::runtime_error(what) {}
};

class EtherNode {
 public:
  EtherNode(std::weak_ptr<EthercatManager> manager, int32_t slave_no)
      : manager_(manager), slave_no_(slave_no) {}
  virtual ~EtherNode() {}

  void InitSlaveStatus() {
    if (manager_.expired()) {
      throw EtherNodeError("Constructed with expired EthercatManager.");
    }
    if (slave_no_ > manager_.lock()->GetSlaveNum()) {
      throw EtherNodeError(
          "slave_no is larger than "
          "ec_slavecount");
    }

    manager_.lock()->GetStatus(slave_no_, slave_name_, slave_configadr_,
                               slave_aliasadr_, slave_eep_man_, slave_eep_id_,
                               slave_eep_rev_, slave_hasdc_, obits_size_,
                               ibits_size_);

    ETHER_INFO("slave_no: {}, name: {}, config addr: {}, alias_addr: {}, eep_man: {}, eep_id: {}, eep_rev: {}, hasdc: {}, obits size: {}, ibits size: {}", slave_no_, slave_name_, slave_configadr_,
               slave_aliasadr_, slave_eep_man_, slave_eep_id_,
               slave_eep_rev_, slave_hasdc_, obits_size_,
               ibits_size_);
  }

  int32_t GetSlaveNo() { return slave_no_; }
  std::string GetSlaveName() { return slave_name_; }
  uint16_t GetSlaveConfigadr() { return slave_configadr_; }
  uint16_t GetSlaveAliasadr() { return slave_aliasadr_; }
  uint32_t GetSlaveEepMan() { return slave_eep_man_; }
  uint32_t GetSlaveEepId() { return slave_eep_id_; }
  uint32_t GetSlaveEepRev() { return slave_eep_rev_; }

 public:
  virtual void InitSlaveNode() = 0;
  virtual void GetRawInputs() = 0;
  virtual void GetRawOutputs() = 0;
  virtual void WriteRawOutputs() = 0;

 protected:
  /** ethercat manager */
  std::weak_ptr<EthercatManager> manager_;
  /** slave no */
  const int32_t slave_no_;
  /** readable name */
  std::string slave_name_;
  /** Configured address */
  uint16_t slave_configadr_;
  /** Alias address */
  uint16_t slave_aliasadr_;
  /** Manufacturer from EEprom */
  uint32_t slave_eep_man_;
  /** ID from EEprom */
  uint32_t slave_eep_id_;
  /** revision from EEprom */
  uint32_t slave_eep_rev_;
  /** has DC capability */
  uint8_t slave_hasdc_;
  /** Obits size */
  uint16_t obits_size_;
  /** Ibits size */
  uint16_t ibits_size_;
};

}  // namespace ether_control