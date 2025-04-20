/*
  Copyright (c) 2024 XDU-IRobot

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#ifndef LIBRM_MODULES_ALGORITHM_THRESHOLD_TRIGGER_HPP
#define LIBRM_MODULES_ALGORITHM_THRESHOLD_TRIGGER_HPP

#include <functional>
#include <vector>

namespace rm::modules::algorithm {

/**
 * @brief 阈值触发器。Attach callbacks to rising and falling edges of a variable.
 * @brief https://github.com/lunarifish/edge-trigger
 */
template <typename T>
class ThresholdTrigger {
 public:
  ThresholdTrigger(T& variable, T rising_threshold, T falling_threshold)
      : variable_(variable), rising_threshold_(rising_threshold), falling_threshold_(falling_threshold) {
    if (variable > rising_threshold) {
      state_ = true;
    } else {
      state_ = false;
    }
  }

  template <typename U = T, std::enable_if_t<std::is_same_v<U, bool>, int> = 0>
  explicit ThresholdTrigger(U& var) : variable_(var), rising_threshold_(true), falling_threshold_(false) {
    if (variable_) {
      state_ = true;
    } else {
      state_ = false;
    }
  }

  ThresholdTrigger& OnRising(const std::function<void()>& callback) {
    rising_edge_callbacks_.push_back(callback);
    return *this;
  }

  ThresholdTrigger& OnFalling(const std::function<void()>& callback) {
    falling_edge_callbacks_.push_back(callback);
    return *this;
  }

  void Update() {
    if (variable_ >= rising_threshold_ && !state_) {
      state_ = true;
      for (const auto& callback : rising_edge_callbacks_) {
        callback();
      }
    } else if (variable_ <= falling_threshold_ && state_) {
      state_ = false;
      for (const auto& callback : falling_edge_callbacks_) {
        callback();
      }
    }
  }

 private:
  bool state_;
  T& variable_;
  T rising_threshold_;
  T falling_threshold_;
  std::vector<std::function<void()>> rising_edge_callbacks_{};
  std::vector<std::function<void()>> falling_edge_callbacks_{};
};

}  // namespace rm::modules::algorithm

#endif  // LIBRM_MODULES_ALGORITHM_THRESHOLD_TRIGGER_HPP