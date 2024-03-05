/*
 * @Descripttion:
 * @Author: editorxu
 * @version:
 * @Date: 2024-03-04 16:23:34
 * @LastEditors: editorxu
 * @LastEditTime: 2024-03-05 11:45:27
 */
#include "command.h"

#include <algorithm>
#include <iostream>
#include <sstream>

#include "backend.h"

namespace ether_backend {

const std::unordered_set<std::string> Command::m_validCommand = {
    "help", "register", "master", "slave", "pdo", "print"};

Command::Command() {}
Command::~Command() {}

void Command::Run() {
  std::cout << "SOEM-master Version 1.03" << std::endl;

  for (;;) {
    try {
      do {
        m_command.clear();
        m_parameters.clear();

        GetCommand();
        if (ValidCommand()) {
          ExecuteCommand();
        }

      } while (m_command != "quit");

      break;
    } catch (const std::exception &e) {
      std::cout << e.what() << std::endl;
    }
  }
}

void Command::GetCommand() {
  std::cout << ">> ";
  std::string readString = "";
  getline(std::cin, readString);
  std::istringstream iss(readString);

  iss >> std::ws;
  iss >> m_command;

  std::transform(m_command.begin(), m_command.end(), m_command.begin(),
                 ::tolower);

  while (!iss.eof()) {

    std::string parameter;
    iss >> parameter;
    if (!parameter.empty())
      m_parameters.emplace_back(parameter);
  }
}

bool Command::ValidCommand() {
  if (m_validCommand.find(m_command) == m_validCommand.end()) {
    std::cout << "Invalid Command: " << m_command << std::endl;
    return false;
  }
  return true;
}

void Command::PrintHelp(const std::vector<std::string> &paramters) {
  std::cout << "help  : Help." << std::endl
            << "print : Print info." << std::endl
            << "register : register slave node with node id." << std::endl
            << "master : master control." << std::endl
            << "slave : slave control." << std::endl
            << "pdo : pdo control." << std::endl
            << "quit  : Quit" << std::endl;
}

void Command::MasterControl(const std::vector<std::string> &paramters) {
  if (paramters.empty())
    return;
  std::string subCmd = paramters[0];
  if (subCmd == "pre_safe_op") {
    EnablePreSafeOP();
  } else if (subCmd == "config_slave") {
    ConfigSlaveNode();
  } else if (subCmd == "dc") {
    EnableDC();
  } else if (subCmd == "safe_op") {
    EnableSafeOP();
  } else if (subCmd == "op") {
    EnableOP();
  } else if (subCmd == "start") {
    StartEther();
  } else if (subCmd == "stop") {
    StopEther();
  } else if (subCmd == "init") {
    EnablePreSafeOP();
    ConfigSlaveNode();
    EnableDC();
    EnableSafeOP();
    EnableOP();
  }
}

void Command::SlaveControl(const std::vector<std::string> &paramters) {
  if (paramters.empty())
    return;
  std::string subCmd = paramters[0];
  if (subCmd == "set" && paramters.size() == 4) {
    int32_t slave_no = std::stoi(paramters[1]);
    std::string field_name = paramters[2];
    std::string strVal = paramters[3];
    SetEtherNodeData(slave_no, field_name, strVal);
  } else if (subCmd == "get" && paramters.size() == 3) {
    int32_t slave_no = std::stoi(paramters[1]);
    std::string field_name = paramters[2];
    std::string data = GetEtherNodeData(slave_no, field_name);
    std::cout << data << std::endl;
  }
}

void Command::RegisterSlave(const std::vector<std::string> &paramters) {
  if (paramters.empty())
    return;
  std::string subCmd = paramters[0];

  int32_t slave_no = std::stoi(subCmd);
  RegisterEtherNode(slave_no);
}

void Command::PdoControl(const std::vector<std::string> &paramters) {
  if (paramters.empty())
    return;
  std::string subCmd = paramters[0];
  if (subCmd == "rx" && paramters.size() == 4) {
    int32_t slave_no = std::stoi(paramters[1]);
    uint32_t index = std::stoul(paramters[2]);
    uint16_t addr = std::stoul(paramters[3], 0, 16);
    SetEtherNodeRxPDOMap(slave_no, index, addr);
  } else if (subCmd == "tx" && paramters.size() == 4) {
    int32_t slave_no = std::stoi(paramters[1]);
    uint32_t index = std::stoul(paramters[2]);
    uint16_t addr = std::stoul(paramters[3], 0, 16);
    SetEtherNodeTxPDOMap(slave_no, index, addr);
  } else {
    std::cout << "please check your command." << std::endl;
  }
}

void Command::PrintInfo(const std::vector<std::string> &paramters) {
  if (paramters.empty())
    return;
  std::string subCmd = paramters[0];
  if (subCmd == "slavelist") {
    std::cout << PrintEtherNodeList() << std::endl;
  } else if (subCmd == "pdomap" && paramters.size() == 2) {
    int32_t slave_no = std::stoi(paramters[1]);
    std::cout << PrintEtherPDOMap(slave_no) << std::endl;
  } else {
    std::cout << "please check your command." << std::endl;
  }
}

std::map<std::string, CmdHandleFunc> Command::m_handle_map = {
    {"help", Command::PrintHelp},       {"register", Command::RegisterSlave},
    {"master", Command::MasterControl}, {"slave", Command::SlaveControl},
    {"pdo", Command::PdoControl},       {"print", Command::PrintInfo},
};

void Command::ExecuteCommand() {
  if (m_handle_map.find(m_command) != m_handle_map.end()) {
    auto func = m_handle_map[m_command];
    func(m_parameters);
  }
  return;
}
} // namespace ether_backend