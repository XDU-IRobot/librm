/*
  Copyright (c) 2025 XDU-IRobot

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

/**
 * @file  librm/modules/rgb_led_controller.hpp
 * @brief RGB LED控制器
 */

#ifndef LIBRM_MODULES_RGB_LED_CONTROLLER_HPP
#define LIBRM_MODULES_RGB_LED_CONTROLLER_HPP

#include <tuple>
#include <type_traits>

#include "librm/core/typedefs.hpp"

namespace rm::modules {

/**
 * @brief RGB LED灯效的基类，子类通过继承该类并且实现Update和Reset方法来定义一种灯效
 */
class RgbLedPattern {
 public:
  virtual ~RgbLedPattern() = default;
  using Rgb = std::tuple<u8, u8, u8>;  ///< (0, 0, 0) ~ (255, 255, 255)

  virtual Rgb Update() = 0;
  virtual void Reset() = 0;
};

/**
 * @brief 几个常用的LED灯效
 * @note  注意这几个灯效都是按1ms一个step来设计的，如果效果看起来不是很正常，检查一下Update调用的频率是否正确
 */
namespace led_pattern {

/**
 * @brief LED灯关闭
 */
class Off : public RgbLedPattern {
 public:
  Off() = default;

  Rgb Update() override { return Rgb(0, 0, 0); }

  void Reset() override {}
};

/**
 * @brief 绿色呼吸灯
 */
class GreenBreath : public RgbLedPattern {
 public:
  GreenBreath() = default;

  Rgb Update() override {
    if (increasing_) {
      brightness_ += 0.3f;
      if (brightness_ >= 255.f) {
        brightness_ = 255.f;
        increasing_ = false;
      }
    } else {
      brightness_ -= 0.3f;
      if (brightness_ <= 0.f) {
        brightness_ = 0.f;
        increasing_ = true;
      }
    }
    return Rgb(0, brightness_, 0);
  }

  void Reset() override {
    brightness_ = 0.f;
    increasing_ = true;
  }

 private:
  f32 brightness_{0.f};
  bool increasing_{true};
};

/**
 * @brief 红灯闪烁
 */
class RedFlash : public RgbLedPattern {
 public:
  RedFlash() = default;

  Rgb Update() override {
    flash_counter_ = (flash_counter_ + 1) % 1000;
    if (flash_counter_ < 500) {
      return Rgb(255, 0, 0);
    }
    return Rgb(0, 0, 0);
  }

  void Reset() override { flash_counter_ = 0; }

 private:
  usize flash_counter_{0};
};

/**
 * @brief RGB流动
 */
class RgbFlow : public RgbLedPattern {
 public:
  RgbFlow() = default;

  Rgb Update() override {
    phase_ = (phase_ + 1) % 1536;
    u8 r, g, b;
    if (phase_ < 512) {
      r = 255 - (phase_ * 255) / 512;
      g = (phase_ * 255) / 512;
      b = 0;
    } else if (phase_ < 1024) {
      r = 0;
      g = 255 - ((phase_ - 512) * 255) / 512;
      b = ((phase_ - 512) * 255) / 512;
    } else {
      r = ((phase_ - 1024) * 255) / 512;
      g = 0;
      b = 255 - ((phase_ - 1024) * 255) / 512;
    }
    return Rgb(r, g, b);
  }

  void Reset() override { phase_ = 0; }

 private:
  usize phase_{0};
};

// NOTE: 你可以参照上面的三个Pattern，通过继承RgbLedPattern类来实现自己的LED灯效

}  // namespace led_pattern

namespace detail {
template <typename T, typename Tuple>
struct tuple_element_index;

template <typename T, typename... Ts>
struct tuple_element_index<T, std::tuple<T, Ts...>> {
  static constexpr usize value = 0;
};

template <typename T, typename U, typename... Ts>
struct tuple_element_index<T, std::tuple<U, Ts...>> {
  static constexpr usize value = 1 + tuple_element_index<T, std::tuple<Ts...>>::value;
};
}  // namespace detail

template <typename... PatternTypes>
class RgbLedController {
  static_assert(sizeof...(PatternTypes) > 0, "At least one pattern type must be provided");
  static_assert((std::is_base_of<RgbLedPattern, PatternTypes>::value && ...),
                "All PatternTypes must derive from RgbLedPattern");

 public:
  RgbLedController() : patterns_{PatternTypes()...} {}

  /**
   * @brief           切换当前的灯效
   * @tparam Pattern  要切换到的灯效
   */
  template <typename Pattern>
  void SetPattern() {
    constexpr usize pattern_id = detail::tuple_element_index<Pattern, std::tuple<PatternTypes...>>::value;
    current_pattern_index_ = pattern_id;
    std::get<pattern_id>(patterns_).Reset();
  }

  /**
   * @brief  更新当前灯效并获取当前颜色
   * @return 当前颜色的RGB值
   */
  std::tuple<u8, u8, u8> Update() {
    current_color_ = std::apply(
        [this](auto &...patterns) {
          RgbLedPattern::Rgb result;
          usize i = 0;
          // 使用折叠表达式遍历元组中的所有灯效模式
          // 当 i 等于当前灯效模式索引时，调用其 Update 方法
          (
              [&] {
                if (i++ == current_pattern_index_) {
                  result = patterns.Update();
                }
              }(),
              ...);
          return result;
        },
        patterns_);
    return current_color_;
  }

  /**
   * @brief  获取当前颜色
   * @return 当前颜色的RGB值
   */
  const auto &current_color() const { return current_color_; }

 private:
  RgbLedPattern::Rgb current_color_{0, 0, 0};
  std::tuple<PatternTypes...> patterns_;
  usize current_pattern_index_{0};
};

}  // namespace rm::modules

#endif