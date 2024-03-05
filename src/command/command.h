/*
 * @Descripttion:
 * @Author: editorxu
 * @version:
 * @Date: 2024-03-04 16:23:52
 * @LastEditors: editorxu
 * @LastEditTime: 2024-03-05 10:41:31
 */
#pragma once

#include <map>
#include <string>
#include <unordered_set>
#include <vector>

#include <functional>

namespace ether_backend {
using CmdHandleFunc = std::function<void(const std::vector<std::string> &)>;

class Command {
public:
  Command();
  ~Command();
  void Run();

public:
  void GetCommand();
  bool ValidCommand();
  void ExecuteCommand();

public:
  static void PrintHelp(const std::vector<std::string> &paramters);
  static void MasterControl(const std::vector<std::string> &paramters);
  static void SlaveControl(const std::vector<std::string> &paramters);
  static void RegisterSlave(const std::vector<std::string> &paramters);
  static void PdoControl(const std::vector<std::string> &paramters);
  static void PrintInfo(const std::vector<std::string> &paramters);

private:
  std::string m_command;
  std::vector<std::string> m_parameters;

  static std::map<std::string, CmdHandleFunc> m_handle_map;
  static const std::unordered_set<std::string> m_validCommand;
};

} // namespace ether_backend