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
 */
namespace led_pattern {

/**
 * @brief LED灯关闭
 */
class Off : public RgbLedPattern {
 public:
  Off() = default;

  Rgb Update(TimePoint now) override { return Rgb(0, 0, 0); }

  void Reset(TimePoint now) override {}
};

/**
 * @brief 绿色呼吸灯
 * @note  呼吸周期约为1700ms（从0到255再回到0）
 */
class GreenBreath : public RgbLedPattern {
 public:
  GreenBreath() = default;

  Rgb Update(TimePoint now) override {
    auto elapsed = ElapsedMs(start_time_, now);
    // 呼吸周期：1700ms
    constexpr u32 kBreathPeriod = 1700;
    u32 phase = elapsed % kBreathPeriod;
    
    f32 brightness;
    if (phase < kBreathPeriod / 2) {
      // 前半周期：从0到255
      brightness = (phase * 255.0f) / (kBreathPeriod / 2);
    } else {
      // 后半周期：从255到0
      brightness = 255.0f - ((phase - kBreathPeriod / 2) * 255.0f) / (kBreathPeriod / 2);
    }
    
    return Rgb(0, static_cast<u8>(brightness), 0);
  }

  void Reset(TimePoint now) override { start_time_ = now; }

 private:
  TimePoint start_time_;
};

/**
 * @brief 红灯闪烁
 * @note  闪烁周期为1000ms（500ms开，500ms关）
 */
class RedFlash : public RgbLedPattern {
 public:
  RedFlash() = default;

  Rgb Update(TimePoint now) override {
    auto elapsed = ElapsedMs(start_time_, now);
    // 闪烁周期：1000ms
    u32 phase = elapsed % 1000;
    if (phase < 500) {
      return Rgb(255, 0, 0);
    }
    return Rgb(0, 0, 0);
  }

  void Reset(TimePoint now) override { start_time_ = now; }

 private:
  TimePoint start_time_;
};

/**
 * @brief RGB流动
 * @note  完整颜色循环周期为1536ms（红->绿->蓝->红）
 */
class RgbFlow : public RgbLedPattern {
 public:
  RgbFlow() = default;

  Rgb Update(TimePoint now) override {
    auto elapsed = ElapsedMs(start_time_, now);
    // 完整颜色循环周期：1536ms
    u32 phase = elapsed % 1536;
    
    u8 r, g, b;
    if (phase < 512) {
      r = 255 - (phase * 255) / 512;
      g = (phase * 255) / 512;
      b = 0;
    } else if (phase < 1024) {
      r = 0;
      g = 255 - ((phase - 512) * 255) / 512;
      b = ((phase - 512) * 255) / 512;
    } else {
      r = ((phase - 1024) * 255) / 512;
      g = 0;
      b = 255 - ((phase - 1024) * 255) / 512;
    }
    return Rgb(r, g, b);
  }

  void Reset(TimePoint now) override { start_time_ = now; }

 private:
  TimePoint start_time_;
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
 * // 在任意频率的定时器或主循环中更新（不要求固定1ms）
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