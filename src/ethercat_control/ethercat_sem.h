/*
 * @Descripttion:
 * @Author: editorxu
 * @version:
 * @Date: 2024-01-12 17:57:10
 * @LastEditors: editorxu
 * @LastEditTime: 2024-01-12 20:38:37
 */
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>

namespace ether_control {
using namespace std::chrono_literals;
class Sem {
 public:
  explicit Sem(int count = 0) : count_(count) {}
  void Notify() {
    std::unique_lock<std::mutex> lock(m_);
    ++count_;
    cv_.notify_one();
  }

  void Wait() {
    std::unique_lock<std::mutex> lock(m_);
    cv_.wait(lock, [this]() -> bool { return count_ > 0; });
    count_--;
  }

  void WaitFor(int idx) {
    std::unique_lock<std::mutex> lock(m_);
    cv_.wait_for(lock, idx * 1us, [this]() -> bool { return count_ > 0; });
    count_--;
  }

  void WaitUntil(std::chrono::time_point<std::chrono::high_resolution_clock> end_time) {
    std::unique_lock<std::mutex> lock(m_);
    cv_.wait_until(lock, end_time, [this]() -> bool { return count_ > 0; });
    count_--;
  }

  int GetCount() { return count_; }

 private:
  std::atomic<int> count_;
  std::mutex m_;
  std::condition_variable cv_;
};  // class Sem
}  // namespace ether_control