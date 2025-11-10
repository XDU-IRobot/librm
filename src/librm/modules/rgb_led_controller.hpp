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
 * @brief RGB LED控制器（基于通用序列播放器实现）
 */

#ifndef LIBRM_MODULES_RGB_LED_CONTROLLER_HPP
#define LIBRM_MODULES_RGB_LED_CONTROLLER_HPP

#include <tuple>

#include "librm/core/typedefs.hpp"
#include "librm/modules/sequence_player.hpp"

namespace rm::modules {

/**
 * @brief RGB LED灯效的基类，继承自SequenceGenerator
 */
class RgbLedPattern : public SequenceGenerator<std::tuple<u8, u8, u8>> {
 public:
  using Rgb = std::tuple<u8, u8, u8>;  ///< (0, 0, 0) ~ (255, 255, 255)
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

/**
 * @brief RGB LED控制器，基于通用的SequencePlayer实现
 * @tparam PatternTypes LED灯效类型列表，所有类型必须继承自RgbLedPattern
 *
 * @example
 * ```cpp
 * // 创建RGB LED控制器，包含多种灯效
 * RgbLedController<led_pattern::Off, led_pattern::GreenBreath, led_pattern::RedFlash> led_controller;
 *
 * // 切换到绿色呼吸灯效
 * led_controller.SetPattern<led_pattern::GreenBreath>();
 *
 * // 在1ms定时器中更新
 * auto [r, g, b] = led_controller.Update();
 * SetRgbLed(r, g, b);
 * ```
 */
template <typename... PatternTypes>
class RgbLedController : public SequencePlayer<RgbLedPattern::Rgb, PatternTypes...> {
 public:
  using Base = SequencePlayer<RgbLedPattern::Rgb, PatternTypes...>;

  /**
   * @brief           切换当前的灯效
   * @tparam Pattern  要切换到的灯效类型
   */
  template <typename Pattern>
  void SetPattern() {
    Base::template SetSequence<Pattern>();
  }

  /**
   * @brief  获取当前颜色（Update方法继承自SequencePlayer）
   * @return 当前颜色的RGB值
   */
  const auto &current_color() const { return Base::current_output(); }
};

}  // namespace rm::modules

#endif