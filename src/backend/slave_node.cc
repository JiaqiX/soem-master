#include "slave_node.h"
#include "string.h"

#include <sstream>
#include "ethercat_utils.h"

#include "log/ethercat_log.h"

namespace ether_backend {

SlaveNode::SlaveNode(std::weak_ptr<EthercatManager> manager, int32_t slave_no)
    : EtherNode(manager, slave_no) {
  ETHER_INFO("node info: slave_no: {}", slave_no_);
}

SlaveNode::~SlaveNode() {}

void SlaveNode::InitSlaveNode() {
  // init slave node
  InitSlaveStatus();
  // map pdo
  InitPDOMap();
  // init data
  inputs.resize(ibits_size_ / 8);
  outputs.resize(obits_size_ / 8);
}

bool SlaveNode::CheckProductCode(uint32_t code) {
  // 校验product id
  if (slave_eep_id_ != (uint32_t)code) {
    ETHER_ERROR("not expected product id: {:08x} != {:08x}, slave no: {}",
                slave_eep_id_, (uint32_t)code, slave_no_);
    return false;
  }
  return true;
}

void SlaveNode::GetRawInputs() {
  if (manager_.expired()) {
    throw SlaveNodeError("Called with expired EthercatManager.");
  }
  std::unique_lock<std::mutex> lock(mtx_input_);
  manager_.lock()->ReadInputs(slave_no_, inputs.data(), inputs.size());
}
void SlaveNode::GetRawOutputs() {
  if (manager_.expired()) {
    throw SlaveNodeError("Called with expired EthercatManager.");
  }
  std::unique_lock<std::mutex> lock(mtx_output_);
  manager_.lock()->ReadOutputs(slave_no_, outputs.data(), outputs.size());
}
void SlaveNode::WriteRawOutputs() {
  if (manager_.expired()) {
    throw SlaveNodeError("Called with expired EthercatManager.");
  }
  std::unique_lock<std::mutex> lock(mtx_output_);
  manager_.lock()->WriteOutputs(slave_no_, outputs.data(), outputs.size());
}

bool SlaveNode::GetInputBool(int32_t offset) {
  return *(bool *)(inputs.data() + offset);
}

int8_t SlaveNode::GetInputInt8(int32_t offset) {
  return *(int8_t *)(inputs.data() + offset);
}
uint8_t SlaveNode::GetInputUint8(int32_t offset) {
  return *(uint8_t *)(inputs.data() + offset);
}
int16_t SlaveNode::GetInputInt16(int32_t offset) {
  return *(int16_t *)(inputs.data() + offset);
}
uint16_t SlaveNode::GetInputUint16(int32_t offset) {
  return *(uint16_t *)(inputs.data() + offset);
}
int32_t SlaveNode::GetInputInt32(int32_t offset) {
  return *(int32_t *)(inputs.data() + offset);
}
uint32_t SlaveNode::GetInputUint32(int32_t offset) {
  return *(uint32_t *)(inputs.data() + offset);
}
int64_t SlaveNode::GetInputInt64(int32_t offset) {
  return *(int64_t *)(inputs.data() + offset);
}
uint64_t SlaveNode::GetInputUint64(int32_t offset) {
  return *(uint64_t *)(inputs.data() + offset);
}

std::string SlaveNode::GetInput(const std::string &field_name) {
  std::stringstream ss;
  if (txpdo_assign_map_.find(field_name) != txpdo_assign_map_.end()) {
    auto &field_info = txpdo_assign_map_[field_name];
    if (field_info.data_type == "BOOLEAN") {
      ss << GetInputBool(field_info.offset);
    } else if (field_info.data_type == "INTEGER8") {
      ss << GetInputInt8(field_info.offset);
    } else if (field_info.data_type == "INTEGER16") {
      ss << GetInputInt16(field_info.offset);
    } else if (field_info.data_type == "INTEGER32") {
      ss << GetInputInt32(field_info.offset);
    } else if (field_info.data_type == "INTEGER64") {
      ss << GetInputInt64(field_info.offset);
    } else if (field_info.data_type == "UNSIGNED8") {
      ss << GetInputUint8(field_info.offset);
    } else if (field_info.data_type == "UNSIGNED16") {
      ss << GetInputUint16(field_info.offset);
    } else if (field_info.data_type == "UNSIGNED32") {
      ss << GetInputUint32(field_info.offset);
    } else if (field_info.data_type == "UNSIGNED64") {
      ss << GetInputUint64(field_info.offset);
    }
  }
  return ss.str();
}

void SlaveNode::SetOutputBool(int32_t offset, bool val) {
  *(bool *)(outputs.data() + offset) = val;
}
void SlaveNode::SetOutputInt8(int32_t offset, int8_t val) {
  *(int8_t *)(outputs.data() + offset) = val;
}
void SlaveNode::SetOutputUint8(int32_t offset, uint8_t val) {
  *(uint8_t *)(outputs.data() + offset) = val;
}
void SlaveNode::SetOutputInt16(int32_t offset, int16_t val) {
  *(int16_t *)(outputs.data() + offset) = val;
}
void SlaveNode::SetOutputUint16(int32_t offset, uint16_t val) {
  *(uint16_t *)(outputs.data() + offset) = val;
}
void SlaveNode::SetOutputInt32(int32_t offset, int32_t val) {
  *(int32_t *)(outputs.data() + offset) = val;
}
void SlaveNode::SetOutputUint32(int32_t offset, uint32_t val) {
  *(uint32_t *)(outputs.data() + offset) = val;
}
void SlaveNode::SetOutputInt64(int32_t offset, int64_t val) {
  *(int64_t *)(outputs.data() + offset) = val;
}
void SlaveNode::SetOutputUint64(int32_t offset, uint64_t val) {
  *(uint64_t *)(outputs.data() + offset) = val;
}

void SlaveNode::SetOutput(const std::string &field_name, std::string strVal) {
  std::stringstream ss(strVal);
  if (txpdo_assign_map_.find(field_name) != txpdo_assign_map_.end()) {
    auto &field_info = txpdo_assign_map_[field_name];
    if (field_info.data_type == "BOOLEAN") {
      bool val;
      ss >> val;
      SetOutputBool(field_info.offset, std::any_cast<bool>(val));
    } else if (field_info.data_type == "INTEGER8") {
      int8_t val;
      ss >> val;
      SetOutputInt8(field_info.offset, std::any_cast<int8_t>(val));
    } else if (field_info.data_type == "INTEGER16") {
      int16_t val;
      ss >> val;
      SetOutputInt16(field_info.offset, std::any_cast<int16_t>(val));
    } else if (field_info.data_type == "INTEGER32") {
      int32_t val;
      ss >> val;
      SetOutputInt32(field_info.offset, std::any_cast<int32_t>(val));
    } else if (field_info.data_type == "INTEGER64") {
      int64_t val;
      ss >> val;
      SetOutputInt64(field_info.offset, std::any_cast<int64_t>(val));
    } else if (field_info.data_type == "UNSIGNED8") {
      uint8_t val;
      ss >> val;
      SetOutputUint8(field_info.offset, std::any_cast<uint8_t>(val));
    } else if (field_info.data_type == "UNSIGNED16") {
      uint16_t val;
      ss >> val;
      SetOutputUint16(field_info.offset, std::any_cast<uint16_t>(val));
    } else if (field_info.data_type == "UNSIGNED32") {
      uint32_t val;
      ss >> val;
      SetOutputUint32(field_info.offset, std::any_cast<uint32_t>(val));
    } else if (field_info.data_type == "UNSIGNED64") {
      uint64_t val;
      ss >> val;
      SetOutputUint64(field_info.offset, std::any_cast<uint64_t>(val));
    }
  }
}

std::string SlaveNode::dtype2string(uint16_t dtype) {
  static std::map<uint16_t, std::string> data_type_map = {
      {ECT_BOOLEAN, "BOOLEAN"},
      {ECT_INTEGER8, "INTEGER8"},
      {ECT_INTEGER16, "INTEGER16"},
      {ECT_INTEGER32, "INTEGER32"},
      {ECT_INTEGER24, "INTEGER24"},
      {ECT_INTEGER64, "INTEGER64"},
      {ECT_UNSIGNED8, "UNSIGNED8"},
      {ECT_UNSIGNED16, "UNSIGNED16"},
      {ECT_UNSIGNED32, "UNSIGNED32"},
      {ECT_UNSIGNED24, "UNSIGNED24"},
      {ECT_UNSIGNED64, "UNSIGNED64"},
      {ECT_REAL32, "REAL32"},
      {ECT_REAL64, "REAL64"},
      {ECT_BIT1, "BIT1"},
      {ECT_BIT2, "BIT2"},
      {ECT_BIT3, "BIT3"},
      {ECT_BIT4, "BIT4"},
      {ECT_BIT5, "BIT5"},
      {ECT_BIT6, "BIT6"},
      {ECT_BIT7, "BIT7"},
      {ECT_BIT8, "BIT8"},
      {ECT_VISIBLE_STRING, "VISIBLE_STRING"},
      {ECT_OCTET_STRING, "OCTET_STRING"},
  };
  if (data_type_map.find(dtype) != data_type_map.end())
    return data_type_map[dtype];
  return std::string("Type 0x%4.4X", dtype);
}

int SlaveNode::InitPDOMap() {
  int wkc, rdl;
  int retVal = 0;
  uint8 nSM, iSM, tSM;
  int Tsize, outputs_bo, inputs_bo;
  uint8 SMt_bug_add;

  SMt_bug_add = 0;
  outputs_bo = 0;
  inputs_bo = 0;
  rdl = sizeof(nSM);
  nSM = 0;
  /* read SyncManager Communication Type object count */
  wkc = ec_SDOread(slave_no_, ECT_SDO_SMCOMMTYPE, 0x00, FALSE, &rdl, &nSM,
                   EC_TIMEOUTRXM);
  /* positive result from slave ? */
  if ((wkc > 0) && (nSM > 2)) {
    /* make nSM equal to number of defined SM */
    nSM--;
    /* limit to maximum number of SM defined, if true the slave can't be
     * configured */
    if (nSM > EC_MAXSM)
      nSM = EC_MAXSM;
    /* iterate for every SM type defined */
    for (iSM = 2; iSM <= nSM; iSM++) {
      rdl = sizeof(tSM);
      tSM = 0;
      /* read SyncManager Communication Type */
      wkc = ec_SDOread(slave_no_, ECT_SDO_SMCOMMTYPE, iSM + 1, FALSE, &rdl,
                       &tSM, EC_TIMEOUTRXM);
      if (wkc > 0) {
        if ((iSM == 2) &&
            (tSM ==
             2))  // SM2 has type 2 == mailbox out, this is a bug in the slave!
        {
          SMt_bug_add = 1;  // try to correct, this works if the types are 0 1 2
                            // 3 and should be 1 2 3 4
        }
        if (tSM)
          tSM += SMt_bug_add;  // only add if SMt > 0

        if (tSM == 3)  // outputs
        {
          /* read the assign RXPDO */
          Tsize = PDOAssign(tSM, ECT_SDO_PDOASSIGN + iSM,
                            (int)(ec_slave[slave_no_].outputs -
                                  (uint8 *)&manager_.lock()->iomap_[0]),
                            outputs_bo);
          outputs_bo += Tsize;
        }
        if (tSM == 4)  // inputs
        {
          /* read the assign TXPDO */
          Tsize = PDOAssign(tSM, ECT_SDO_PDOASSIGN + iSM,
                            (int)(ec_slave[slave_no_].inputs -
                                  (uint8 *)&manager_.lock()->iomap_[0]),
                            inputs_bo);
          inputs_bo += Tsize;
        }
      }
    }
  }

  /* found some I/O bits ? */
  if ((outputs_bo > 0) || (inputs_bo > 0))
    retVal = 1;
  return retVal;
}

int SlaveNode::PDOAssign(uint8_t tSM, uint16_t PDOassign, int mapoffset,
                         int bitoffset) {
  uint16_t idxloop, nidx, subidxloop, rdat, idx, subidx;
  uint8 subcnt;
  int wkc, bsize = 0, rdl;
  int32 rdat2;
  uint8 bitlen, obj_subidx;
  uint16_t obj_idx;
  int abs_offset, abs_bit;
  int relative_offset;

  rdl = sizeof(rdat);
  rdat = 0;
  /* read PDO assign subindex 0 ( = number of PDO's) */
  wkc =
      ec_SDOread(slave_no_, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
  rdat = etohs(rdat);
  /* positive result from slave ? */
  if ((wkc > 0) && (rdat > 0)) {
    /* number of available sub indexes */
    nidx = rdat;
    bsize = 0;
    /* read all PDO's */
    for (idxloop = 1; idxloop <= nidx; idxloop++) {
      rdl = sizeof(rdat);
      rdat = 0;
      /* read PDO assign */
      wkc = ec_SDOread(slave_no_, PDOassign, (uint8)idxloop, FALSE, &rdl, &rdat,
                       EC_TIMEOUTRXM);
      /* result is index of PDO */
      idx = etohs(rdat);
      if (idx > 0) {
        rdl = sizeof(subcnt);
        subcnt = 0;
        /* read number of subindexes of PDO */
        wkc = ec_SDOread(slave_no_, idx, 0x00, FALSE, &rdl, &subcnt,
                         EC_TIMEOUTRXM);
        subidx = subcnt;
        /* for each subindex */
        for (subidxloop = 1; subidxloop <= subidx; subidxloop++) {
          rdl = sizeof(rdat2);
          rdat2 = 0;
          /* read SDO that is mapped in PDO */
          wkc = ec_SDOread(slave_no_, idx, (uint8)subidxloop, FALSE, &rdl,
                           &rdat2, EC_TIMEOUTRXM);
          rdat2 = etohl(rdat2);
          /* extract bitlength of SDO */
          bitlen = LO_BYTE(rdat2);
          bsize += bitlen;
          obj_idx = (uint16_t)(rdat2 >> 16);
          obj_subidx = (uint8)((rdat2 >> 8) & 0x000000ff);
          abs_offset = mapoffset + (bitoffset / 8);
          relative_offset = bitoffset / 8;
          abs_bit = bitoffset % 8;
          ODlist.Slave = slave_no_;
          ODlist.Index[0] = obj_idx;
          OElist.Entries = 0;
          wkc = 0;
          /* read object entry from dictionary if not a filler (0x0000:0x00) */
          if (obj_idx || obj_subidx)
            wkc = ec_readOEsingle(0, obj_subidx, &ODlist, &OElist);

          std::string data_name = OElist.Name[obj_subidx];

          DataInfo info;
          info.name = data_name;
          info.index = obj_idx;
          info.sub_index = obj_subidx;
          info.bit_len = bitlen;
          info.data_type = dtype2string(OElist.DataType[obj_subidx]);
          info.offset = relative_offset;

          if (tSM == 3) {
            rxpdo_assign_map_[data_name] = info;
            rxpdo_assign_vec_.push_back(info);
          } else if (tSM == 4) {
            txpdo_assign_map_[data_name] = info;
            txpdo_assign_vec_.push_back(info);
          }
          bitoffset += bitlen;
        };
      };
    };
  };
  /* return total found bitlength (PDO) */
  return bsize;
}

}  // namespace ether_backend