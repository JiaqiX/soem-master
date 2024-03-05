
#include "ethercat_manager.h"
///
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <chrono>
#include <set>
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

#include "ethercat_utils.h"

#include "log/ethercat_log.h"

namespace ether_control {

static const unsigned EC_TIMEOUTMON = 500;

EthercatManager::EthercatManager(const std::string &ifname,
                                 const std::vector<int32_t> &exclude_slave_list,
                                 bool dc_enable, uint64_t period)
    : ifname_(ifname), dc_enable_(dc_enable), period_(period) {
  std::for_each(
      exclude_slave_list.begin(), exclude_slave_list.end(),
      [this](int slave_no) { this->exclude_slave_set_.insert(slave_no); });
}

EthercatManager::~EthercatManager() {
  if (stop_flag_.load()) {
    ReleaseMasterNode();
  }
}

void EthercatManager::InitEthercatManager() {
  if (InitSoem()) {
    ETHER_INFO("period_: {}ns, dc_enable_: {}.", period_, dc_enable_);
  } else {
    throw EthercatError("Could not initialize SOEM");
  }
}

int EthercatManager::GetSlaveNum() const { return slave_num_; }

void EthercatManager::GetStatus(int32_t slave_no, std::string &name,
                                uint16_t &configadr, uint16_t &aliasadr,
                                uint32_t &eep_man, uint32_t &eep_id,
                                uint32_t &eep_rev, uint8_t &hasdc,
                                uint16_t &obits_size,
                                uint16_t &ibits_size) const {
  if (slave_no > ec_slavecount) {
    return;
  }
  name = std::string(ec_slave[slave_no].name);
  configadr = ec_slave[slave_no].configadr;
  aliasadr = ec_slave[slave_no].aliasadr;
  eep_man = (uint32_t)ec_slave[slave_no].eep_man;
  eep_id = (uint32_t)ec_slave[slave_no].eep_id;
  eep_rev = (uint32_t)ec_slave[slave_no].eep_rev;
  hasdc = ec_slave[slave_no].hasdc;
}

bool EthercatManager::InitMasterNode() {
  // Copy string contents because SOEM library doesn't
  // practice const correctness
  const static unsigned MAX_BUFF_SIZE = 1024;
  char buffer[MAX_BUFF_SIZE];
  size_t name_size = ifname_.size();
  if (name_size > sizeof(buffer) - 1) {
    ETHER_ERROR("Ifname {} exceeds "
                "maximum size of {} bytes",
                ifname_.c_str(), MAX_BUFF_SIZE);
    return false;
  }
  strncpy(buffer, ifname_.c_str(), MAX_BUFF_SIZE);
  ETHER_INFO("ifname_: {}", buffer);

  if (!ec_init(buffer)) {
    ETHER_ERROR("Could not initialize "
                "ethercat driver, "
                "maybe No socket connection on {} or Should Execute as root",
                buffer);
    return false;
  }

  /* find and auto-config slaves */
  if (ec_config_init(FALSE) <= 0) {
    ETHER_ERROR("No slaves found on {}", ifname_);
    return false;
  }
  ETHER_INFO("SOEM found and configured {} slaves", ec_slavecount);

  for (int cnt = 1; cnt <= ec_slavecount; cnt++) {
    ETHER_INFO("slave no: {}, Man: {:08x} ID: {:08x} Rev: {:08x}", cnt,
               (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id,
               (int)ec_slave[cnt].eep_rev);
    slave_num_++;
    if (exclude_slave_set_.count(cnt)) {
      continue;
    }
    slave_list_.emplace_back(cnt);
  }
  ETHER_INFO("Found {} Wanted Drivers, Total {} Drivers", slave_list_.size(),
             slave_num_);
  return true;
}

void EthercatManager::ReleaseMasterNode() {
  // Request init operational state for all slaves
  ec_slave[0].state = EC_STATE_INIT;
  /* request init state for all slaves */
  ec_writestate(0);
  // stop SOEM, close socket
  ec_close();
}

bool EthercatManager::EnablePreSafeOP() {
  ETHER_INFO("enable pre safe op.");
  for (auto cnt : slave_list_) {
    if (ec_statecheck(cnt, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4) !=
        EC_STATE_PRE_OP) {
      ETHER_ERROR("Could not set "
                  "EC_STATE_PRE_OP. slave no: {}",
                  cnt);
      return false;
    }
  }
  return true;
}
bool EthercatManager::ConfigSlaveNode() {
  ETHER_INFO("config slave node.");

  SlaveConfig();

  // configure IOMap
  int iomap_size = ec_config_map(iomap_);
  ETHER_INFO("SOEM IOMap size: {}", iomap_size);
  return true;
}
bool EthercatManager::EnableDC() {
  // locates dc slaves
  if (dc_enable_) {
    ETHER_INFO("enable dc.");
    ec_configdc();
    for (int cnt = 1; cnt <= ec_slavecount; cnt++) {
      ec_dcsync0(cnt, TRUE, period_, 0);
    }
  }
  return true;
}
bool EthercatManager::EnableSafeOP() {
  ETHER_INFO("enable safe op.");
  for (auto cnt : slave_list_) {
    if (ec_statecheck(cnt, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) !=
        EC_STATE_SAFE_OP) {
      ETHER_ERROR("Could not set "
                  "EC_STATE_SAFE_OP. slave no: {}",
                  cnt);
      return false;
    }
  }
  return true;
}

bool EthercatManager::EnableOP() {
  ETHER_INFO("enable op.");
  expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC -
                 exclude_slave_set_.size();

  /*
    This section attempts to bring all slaves to operational status. It does so
    by attempting to set the status of all slaves (ec_slave[0]) to operational,
    then proceeding through 40 send/recieve cycles each waiting up to 50 ms for
    a response about the status.
  */
  for (auto cnt : slave_list_) {
    ec_slave[cnt].state = EC_STATE_OPERATIONAL;
  }
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);

  for (auto cnt : slave_list_) {
    ec_writestate(cnt);
  }
  int chk = 40;
  do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    for (auto cnt : slave_list_) {
      ec_statecheck(cnt, EC_STATE_OPERATIONAL,
                    20000); // 20 ms wait for state check
    }
  } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  for (auto cnt : slave_list_) {
    if (ec_statecheck(cnt, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) !=
        EC_STATE_OPERATIONAL) {
      ETHER_ERROR("OPERATIONAL state "
                  "not set. slave no: {}",
                  cnt);
      return false;
    }
  }
  ec_readstate();
  for (int cnt = 1; cnt <= ec_slavecount; cnt++) {
    ETHER_INFO(
        "Slave:{}\n Name:{}\n Output size: {}bits\n Input size: {}bits\n "
        "State: {}\n Delay: {}[ns]\n Has DC: {}\n config address: {}\n alias "
        "address: {}",
        cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
        ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc,
        ec_slave[cnt].configadr, ec_slave[cnt].aliasadr);
    if (ec_slave[cnt].hasdc) {
      ETHER_INFO(" DCParentport:{}", ec_slave[cnt].parentport);
    }
    ETHER_INFO(" Activeports:{}.{}.{}.{}",
               (ec_slave[cnt].activeports & 0x01) > 0,
               (ec_slave[cnt].activeports & 0x02) > 0,
               (ec_slave[cnt].activeports & 0x04) > 0,
               (ec_slave[cnt].activeports & 0x08) > 0);
    ETHER_INFO(" Configured address: {:04x}", ec_slave[cnt].configadr);

    uint16_t sync_mode = ReadSDO<uint16_t>(cnt, 0x1c32, 0x01);
    uint32_t cycle_time = ReadSDO<uint32_t>(cnt, 0x1c32, 0x02);
    uint32_t minimum_cycle_time = ReadSDO<uint32_t>(cnt, 0x1c32, 0x05);
    uint32_t calc_and_copy_time = ReadSDO<uint32_t>(cnt, 0x1c32, 0x06);
    uint32_t sync0_cycle_time = ReadSDO<uint32_t>(cnt, 0x1c32, 0x0a);
    ETHER_INFO("slave {} SM output parameter: PDO syncmode {:2x}, cycle time "
               "{} ns (min {}), sync0 cycle time {} ns, calc and copy time {}",
               cnt, sync_mode, cycle_time, minimum_cycle_time, sync0_cycle_time,
               calc_and_copy_time);
  }

  ETHER_INFO("Finished configuration successfully");
  return true;
}

bool EthercatManager::InitSoem() {
  if (!InitMasterNode())
    return false;
  if (!EnablePreSafeOP())
    return false;
  if (!ConfigSlaveNode())
    return false;
  if (!EnableDC())
    return false;
  if (!EnableSafeOP())
    return false;
  if (!EnableOP())
    return false;

  return true;
}

int64_t EthercatManager::CalcDcPiSync(int64_t _refTime, int64_t _cycleTime,
                                      int64_t _shiftTime) {
  static double kP = 0.05, kI = 0.01;
  static int64_t iTerm = 0;
  int64_t adjTime;
  int32_t iKp = 0, iKpBase, iKi = 0, iKiBase;
  DoubleToFixed(kP, &iKp, &iKpBase);
  DoubleToFixed(kI, &iKi, &iKiBase);

  int64_t error = (_refTime - _shiftTime) % _cycleTime;
  if (error > (_cycleTime / 2))
    error = error - _cycleTime;

  int64_t pTerm = error * iKp;
  iTerm += (error * iKi);

  adjTime = -(pTerm >> iKpBase) - (iTerm >> iKiBase);

  // if (adjTime > _cycleTime / 2) adjTime = _cycleTime / 2;
  // if (adjTime < -_cycleTime / 2) adjTime = -_cycleTime / 2;

  return adjTime;
}

void SetNameForCurrentThread(const std::string &name) {
  if (name.size() < 15) {
    pthread_setname_np(pthread_self(), name.data());
  } else {
    std::string real_name =
        name.substr(0, 8) + ".." + name.substr(name.size() - 5, name.size());
    pthread_setname_np(pthread_self(), real_name.c_str());
  }
}

void EthercatManager::HandleErrors() {
  // set thread name
  std::string name = "etherHandleErrors";
  SetNameForCurrentThread(name);

  std::this_thread::sleep_for(std::chrono::nanoseconds(20 * period_));

  while (!stop_flag_.load()) {
    if (wkc < expected_wkc || ec_group[0].docheckstate) {
      ETHER_ERROR("wkc < expected_wkc || ec_group[0].docheckstate, wkc: {}, "
                  "expected_wkc: {}",
                  wkc, expected_wkc);
      ec_group[0].docheckstate = FALSE;
      ec_readstate();
      for (auto slave : slave_list_) {
        if ((ec_slave[slave].group == 0) &&
            (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
          ec_group[0].docheckstate = TRUE;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            ETHER_ERROR(
                "ERROR : slave {} is in SAFE_OP + ERROR, attempting ack.",
                slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
          } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
            ETHER_ERROR(
                "ERROR : slave {} is in SAFE_OP, change to OPERATIONAL.",
                slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
          } else if (ec_slave[slave].state > 0) {
            ETHER_INFO("MESSAGE : slave {} reconfigured. current state: {:04x}",
                       slave, ec_slave[slave].state);
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              ETHER_INFO(
                  "MESSAGE : slave {} reconfigured. current state: {:04x}",
                  slave, ec_slave[slave].state);
            }
          } else if (!ec_slave[slave].islost) {
            ETHER_WARN("WARN : slave {} re-check state. current state: {:04x}",
                       slave, ec_slave[slave].state);
            /* re-check state */
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (!ec_slave[slave].state) {
              ec_slave[slave].islost = TRUE;
              ETHER_ERROR("ERROR : slave {} lost. current state: {:04x}", slave,
                          ec_slave[slave].state);
            }
          }
        }
        if (ec_slave[slave].islost) {
          ETHER_ERROR("WARN : slave {} lost. current state: {:04x}", slave,
                      ec_slave[slave].state);
          if (!ec_slave[slave].state) {
            if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              ETHER_INFO("MESSAGE : slave {} recovered", slave);
            }
          } else {
            ec_slave[slave].islost = FALSE;
            ETHER_INFO("MESSAGE : slave {} found. current state: {:04x}", slave,
                       ec_slave[slave].state);
          }
        }
      }

      if (!ec_group[0].docheckstate)
        ETHER_INFO("OK : all slaves resumed OPERATIONAL.");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void EthercatManager::LoopWorker() {
  // set thread name
  std::string name = "etherLoopWorker";
  SetNameForCurrentThread(name);

  // For DC computation
  int64 dcRefTime = 0;
  int64 lastDcTime = 0;
  int32 shiftTime = period_ / 2;
  int64 timerOffset = 0;

  auto now_time = std::chrono::high_resolution_clock::now();

  if (dc_enable_) {
    // Update DC time for first time
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    now_time += std::chrono::nanoseconds(10 * period_ - (ec_DCtime % period_));
    // Align to DC clock edge
    std::this_thread::sleep_until(now_time);
  }
  while (!stop_flag_.load()) {
    // expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    {
      sent = ec_send_processdata();
      wkc = ec_receive_processdata(period_ / 2000);
    }

    // 收到数据后，调用回调发布数据
    if (callback_)
      callback_();

    sem_.Notify();

    // Apply offset to threadTimer to Sync with DC clock
    if (dc_enable_) {
      now_time += std::chrono::nanoseconds(period_ + timerOffset);
      std::this_thread::sleep_until(now_time);

      dcRefTime += (ec_DCtime - lastDcTime);
      lastDcTime = ec_DCtime;
      timerOffset = CalcDcPiSync(dcRefTime, period_, shiftTime);
    } else {
      now_time += std::chrono::nanoseconds(period_);
      std::this_thread::sleep_until(now_time);
    }
  }
}

void EthercatManager::ReadInputs(int slave_no, void *inputs, size_t size) {
  memcpy(inputs, ec_slave[slave_no].inputs, size);
}

void EthercatManager::WriteOutputs(int slave_no, const void *outputs,
                                   size_t size) {
  memcpy(ec_slave[slave_no].outputs, outputs, size);
}
void EthercatManager::ReadOutputs(int slave_no, void *outputs, size_t size) {
  memcpy(outputs, ec_slave[slave_no].outputs, size);
}

void EthercatManager::SetSlaveConfig(int slave_idx, PO2SOconfigFunc func) {
  pdo_map_[slave_idx] = func;
}

void EthercatManager::SlaveConfig() {
  for (int slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++) {
    ec_slavet *slave = &ec_slave[slave_idx];
    if (pdo_map_.find(slave_idx) == pdo_map_.end())
      continue;
    slave->PO2SOconfig = pdo_map_[slave_idx];
  }
}

} // namespace ether_control
