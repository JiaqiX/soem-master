/*
 * @Descripttion:
 * @Author: editorxu
 * @version:
 * @Date: 2024-03-01 16:31:01
 * @LastEditors: editorxu
 * @LastEditTime: 2024-03-06 10:00:43
 */
#pragma once

#include <any>
#include <string>

#include "ethercat_control.h"

namespace ether_backend {

extern ether_control::EthercatController g_controller;
extern ether_control::EtherConfig g_config;

//
extern void PreInitEther(const std::string &config_path);
//
extern void InitEther();
//
extern void RegisterEtherNode(int32_t slave_no);
//
extern void StartEther();
//
extern void StopEther();
//
extern std::string PrintEtherNodeList();
//
extern std::string PrintEtherPDOMap(int32_t slave_no);
//
extern void SetEtherNodeData(int32_t slave_no, const std::string &field_name,
                             std::string val);
//
extern std::string GetEtherNodeData(int32_t slave_no,
                                    const std::string &field_name);

}  // namespace ether_backend