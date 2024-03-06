/*
 * @Descripttion:
 * @Author: editorxu
 * @version:
 * @Date: 2024-03-06 09:58:22
 * @LastEditors: editorxu
 * @LastEditTime: 2024-03-06 09:58:27
 */
#pragma once

#include <sstream>
#include <string>

namespace ether_backend {

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
}  // namespace ether_backend