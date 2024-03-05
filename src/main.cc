/*
 * @Descripttion:
 * @Author: editorxu
 * @version:
 * @Date: 2024-03-01 17:41:55
 * @LastEditors: editorxu
 * @LastEditTime: 2024-03-04 16:59:30
 */

#include <stdio.h>
#include <string.h>

#include <iostream>
#include <string>

#include "backend.h"
#include "frontend.h"

#include "command/command.h"

#include "log/ethercat_log.h"

void soem_backend(std::string config_path) {
  ether_backend::InitEtherMaster(config_path);
}

void soem_frontend() {}

int main(int argc, char *argv[]) {
  std::cout << "SOEM (Simple Open EtherCAT master)" << std::endl;

  if (argc > 1) {
    try {
      char config_path[1024];
      strcpy(config_path, argv[1]);
      // Start backend
      soem_backend(config_path);
      // command
      ether_backend::Command cmd;
      cmd.Run();
    } catch (const std::exception &e) {
      std::cout << e.what() << std::endl;
    } catch (...) {
      std::cout << "Unknown exception error" << std::endl;
    }
  } else {
    std::cout << "Usage: soem-master config_path" << std::endl
              << "config_path = ./config.yaml for example" << std::endl;
  }

  std::cout << "End program" << std::endl;
  return 0;
}