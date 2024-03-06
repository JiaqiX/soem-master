/*
 * @Descripttion:
 * @Author: editorxu
 * @version:
 * @Date: 2024-03-04 16:23:34
 * @LastEditors: editorxu
 * @LastEditTime: 2024-03-06 10:49:57
 */
#include "command.h"

#include <algorithm>
#include <iostream>
#include <sstream>

#include "master.h"

namespace ether_backend {

const std::unordered_set<std::string> Command::m_validCommand = {
    "help", "register", "master", "slave", "print"};

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
  if (m_command == "\n" || m_command == "") return false;
  if (m_validCommand.find(m_command) == m_validCommand.end()) {
    std::cout << "Invalid Command: " << m_command << std::endl;
    return false;
  }
  return true;
}

std::map<std::string, CmdHandleFunc> Command::m_handle_map = {
    {"help", Command::PrintHelp},
    {"register", Command::RegisterSlave},
    {"master", Command::MasterControl},
    {"slave", Command::SlaveControl},
    {"print", Command::PrintInfo},
};

void Command::ExecuteCommand() {
  if (m_handle_map.find(m_command) != m_handle_map.end()) {
    auto func = m_handle_map[m_command];
    func(m_parameters);
  }
  return;
}

void Command::PrintHelp(const std::vector<std::string> &paramters) {
  std::cout << "help  : Help." << std::endl
            << "print : Print info." << std::endl
            << "        example: print slavelist   打印扫描出的从站列表" << std::endl
            << "        example: print pdomap 1    打印从站[1]的pdo映射" << std::endl
            << "register : register slave node with node id." << std::endl
            << "        example: register 1        注册从站[1]进行控制,必须在master init之后调用才生效" << std::endl
            << "master : master control." << std::endl
            << "        example: master init       主站进程初始化" << std::endl
            << "        example: master start      启动主站进程" << std::endl
            << "        example: master stop       主站进程退出" << std::endl
            << "slave : slave control." << std::endl
            << "        example: slave pdo get 1 ${fieldname}      获取从站[1]的txpdo字段$field_name数据" << std::endl
            << "        example: slave pdo set 1 ${fieldname} 100  设置从站[1]的rxpdo字段$field_name数据为100" << std::endl
            << "quit  : Quit" << std::endl;
}

void Command::MasterControl(const std::vector<std::string> &paramters) {
  if (paramters.empty())
    return;
  std::string subCmd = paramters[0];
  if (subCmd == "init") {
    InitEther();
  } else if (subCmd == "start") {
    StartEther();
  } else if (subCmd == "stop") {
    StopEther();
    exit(EXIT_SUCCESS);
  } else {
    std::cout << "please check your command." << std::endl;
  }
}

void SlavePDOControl(const std::vector<std::string> &paramters) {
  std::string subCmd = paramters[1];
  if (subCmd == "set" && paramters.size() == 5) {
    int32_t slave_no = std::stoi(paramters[2]);
    std::string field_name = paramters[3];
    std::string strVal = paramters[4];
    SetEtherNodeData(slave_no, field_name, strVal);
  } else if (subCmd == "get" && paramters.size() == 4) {
    int32_t slave_no = std::stoi(paramters[2]);
    std::string field_name = paramters[3];
    std::string data = GetEtherNodeData(slave_no, field_name);
    std::cout << data << std::endl;
  } else {
    std::cout << "please check your command." << std::endl;
  }
}

void SlaveSDOControl(const std::vector<std::string> &paramters) {
}

void Command::SlaveControl(const std::vector<std::string> &paramters) {
  if (paramters.empty())
    return;
  std::string subCmd = paramters[0];
  if (subCmd == "pdo") {
    SlavePDOControl(paramters);
  } else if (subCmd == "sdo") {
    SlaveSDOControl(paramters);
  } else {
    std::cout << "please check your command." << std::endl;
  }
}

void Command::RegisterSlave(const std::vector<std::string> &paramters) {
  if (paramters.empty())
    return;
  std::string subCmd = paramters[0];

  int32_t slave_no = std::stoi(subCmd);
  RegisterEtherNode(slave_no);
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

}  // namespace ether_backend