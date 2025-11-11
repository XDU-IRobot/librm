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
 * @file  librm/modules/buzzer_controller.hpp
 * @brief 蜂鸣器音乐控制器（基于通用序列播放器实现）
 */

#ifndef LIBRM_MODULES_BUZZER_CONTROLLER_HPP
#define LIBRM_MODULES_BUZZER_CONTROLLER_HPP

#include <array>

#include "librm/core/typedefs.hpp"
#include "librm/modules/sequence_player.hpp"

namespace rm::modules {

/**
 * @brief 蜂鸣器音符数据结构
 */
struct BuzzerNote {
  u16 frequency;  ///< 频率 (Hz)，0表示静音
  u16 duration;   ///< 持续时间 (ms)

  constexpr BuzzerNote(u16 freq = 0, u16 dur = 0) : frequency(freq), duration(dur) {}
};

/**
 * @brief 音符频率计算辅助函数（编译期计算十二平均律频率）
 * @param a4_freq A4音符的基准频率（默认440Hz）
 * @param semitones 相对于A4的半音数（正数向上，负数向下）
 * @return 计算得到的频率（Hz）
 * @note 十二平均律公式：f = A4 * 2^(n/12)，其中n为相对于A4的半音数
 */
constexpr u16 CalculateNoteFreq(u16 a4_freq, i16 semitones) {
  // 使用近似计算：2^(n/12) ≈ 使用查找表或者分段计算
  // 为了编译期计算，使用整数运算
  f32 ratio = 1.0f;
  f32 semitone_ratio = 1.059463094359f;  // 2^(1/12)

  if (semitones > 0) {
    for (i16 i = 0; i < semitones; ++i) {
      ratio *= semitone_ratio;
    }
  } else if (semitones < 0) {
    for (i16 i = 0; i > semitones; --i) {
      ratio /= semitone_ratio;
    }
  }

  return static_cast<u16>(a4_freq * ratio + 0.5f);
}

/**
 * @brief 音符频率定义（基于十二平均律计算）
 * @tparam A4Freq A4音符的基准频率（默认440Hz，国际标准音高）
 * @note  使用十二平均律计算所有音符频率
 *        公式：f(n) = A4 * 2^((n-69)/12)，其中n为MIDI音符号
 *        或者：f = A4 * 2^(semitones/12)，semitones为相对于A4的半音数
 *
 * @example
 * ```cpp
 * // 使用标准音高A4=440Hz
 * using Freq = NoteFreq<440>;
 * BuzzerNote note(Freq::kC5, 500);
 *
 * // 使用巴洛克音高A4=415Hz
 * using BaroqueFreq = NoteFreq<415>;
 * BuzzerNote baroque_note(BaroqueFreq::kC5, 500);
 * ```
 */
template <u16 A4Freq = 440>
struct NoteFreq {
  // 静音
  static constexpr u16 kRest = 0;

  // 第3八度（A4下12个半音开始）
  static constexpr u16 kC3 = CalculateNoteFreq(A4Freq, -21);   // A4 - 21半音
  static constexpr u16 kCs3 = CalculateNoteFreq(A4Freq, -20);  // C#3 / Db3
  static constexpr u16 kD3 = CalculateNoteFreq(A4Freq, -19);
  static constexpr u16 kDs3 = CalculateNoteFreq(A4Freq, -18);  // D#3 / Eb3
  static constexpr u16 kE3 = CalculateNoteFreq(A4Freq, -17);
  static constexpr u16 kF3 = CalculateNoteFreq(A4Freq, -16);
  static constexpr u16 kFs3 = CalculateNoteFreq(A4Freq, -15);  // F#3 / Gb3
  static constexpr u16 kG3 = CalculateNoteFreq(A4Freq, -14);
  static constexpr u16 kGs3 = CalculateNoteFreq(A4Freq, -13);  // G#3 / Ab3
  static constexpr u16 kA3 = CalculateNoteFreq(A4Freq, -12);
  static constexpr u16 kAs3 = CalculateNoteFreq(A4Freq, -11);  // A#3 / Bb3
  static constexpr u16 kB3 = CalculateNoteFreq(A4Freq, -10);

  // 第4八度（中央C）
  static constexpr u16 kC4 = CalculateNoteFreq(A4Freq, -9);
  static constexpr u16 kCs4 = CalculateNoteFreq(A4Freq, -8);  // C#4 / Db4
  static constexpr u16 kD4 = CalculateNoteFreq(A4Freq, -7);
  static constexpr u16 kDs4 = CalculateNoteFreq(A4Freq, -6);  // D#4 / Eb4
  static constexpr u16 kE4 = CalculateNoteFreq(A4Freq, -5);
  static constexpr u16 kF4 = CalculateNoteFreq(A4Freq, -4);
  static constexpr u16 kFs4 = CalculateNoteFreq(A4Freq, -3);  // F#4 / Gb4
  static constexpr u16 kG4 = CalculateNoteFreq(A4Freq, -2);
  static constexpr u16 kGs4 = CalculateNoteFreq(A4Freq, -1);  // G#4 / Ab4
  static constexpr u16 kA4 = A4Freq;                          // 基准音
  static constexpr u16 kAs4 = CalculateNoteFreq(A4Freq, 1);   // A#4 / Bb4
  static constexpr u16 kB4 = CalculateNoteFreq(A4Freq, 2);

  // 第5八度
  static constexpr u16 kC5 = CalculateNoteFreq(A4Freq, 3);
  static constexpr u16 kCs5 = CalculateNoteFreq(A4Freq, 4);  // C#5 / Db5
  static constexpr u16 kD5 = CalculateNoteFreq(A4Freq, 5);
  static constexpr u16 kDs5 = CalculateNoteFreq(A4Freq, 6);  // D#5 / Eb5
  static constexpr u16 kE5 = CalculateNoteFreq(A4Freq, 7);
  static constexpr u16 kF5 = CalculateNoteFreq(A4Freq, 8);
  static constexpr u16 kFs5 = CalculateNoteFreq(A4Freq, 9);  // F#5 / Gb5
  static constexpr u16 kG5 = CalculateNoteFreq(A4Freq, 10);
  static constexpr u16 kGs5 = CalculateNoteFreq(A4Freq, 11);  // G#5 / Ab5
  static constexpr u16 kA5 = CalculateNoteFreq(A4Freq, 12);
  static constexpr u16 kAs5 = CalculateNoteFreq(A4Freq, 13);  // A#5 / Bb5
  static constexpr u16 kB5 = CalculateNoteFreq(A4Freq, 14);

  // 第6八度
  static constexpr u16 kC6 = CalculateNoteFreq(A4Freq, 15);
  static constexpr u16 kCs6 = CalculateNoteFreq(A4Freq, 16);  // C#6 / Db6
  static constexpr u16 kD6 = CalculateNoteFreq(A4Freq, 17);
  static constexpr u16 kDs6 = CalculateNoteFreq(A4Freq, 18);  // D#6 / Eb6
  static constexpr u16 kE6 = CalculateNoteFreq(A4Freq, 19);
  static constexpr u16 kF6 = CalculateNoteFreq(A4Freq, 20);
  static constexpr u16 kFs6 = CalculateNoteFreq(A4Freq, 21);  // F#6 / Gb6
  static constexpr u16 kG6 = CalculateNoteFreq(A4Freq, 22);
  static constexpr u16 kGs6 = CalculateNoteFreq(A4Freq, 23);  // G#6 / Ab6
  static constexpr u16 kA6 = CalculateNoteFreq(A4Freq, 24);
  static constexpr u16 kAs6 = CalculateNoteFreq(A4Freq, 25);  // A#6 / Bb6
  static constexpr u16 kB6 = CalculateNoteFreq(A4Freq, 26);

  // 第7八度（可选，高音区）
  static constexpr u16 kC7 = CalculateNoteFreq(A4Freq, 27);
  static constexpr u16 kCs7 = CalculateNoteFreq(A4Freq, 28);  // C#7 / Db7
  static constexpr u16 kD7 = CalculateNoteFreq(A4Freq, 29);
  static constexpr u16 kDs7 = CalculateNoteFreq(A4Freq, 30);  // D#7 / Eb7
  static constexpr u16 kE7 = CalculateNoteFreq(A4Freq, 31);
  static constexpr u16 kF7 = CalculateNoteFreq(A4Freq, 32);
  static constexpr u16 kFs7 = CalculateNoteFreq(A4Freq, 33);  // F#7 / Gb7
  static constexpr u16 kG7 = CalculateNoteFreq(A4Freq, 34);
  static constexpr u16 kGs7 = CalculateNoteFreq(A4Freq, 35);  // G#7 / Ab7
  static constexpr u16 kA7 = CalculateNoteFreq(A4Freq, 36);
  static constexpr u16 kAs7 = CalculateNoteFreq(A4Freq, 37);  // A#7 / Bb7
  static constexpr u16 kB7 = CalculateNoteFreq(A4Freq, 38);
};

// 常用音高标准的别名
using NoteFreqStandard = NoteFreq<440>;  ///< 国际标准音高 A4=440Hz
using NoteFreqBaroque = NoteFreq<415>;   ///< 巴洛克音高 A4=415Hz
using NoteFreqChamber = NoteFreq<432>;   ///< 室内乐音高 A4=432Hz
using NoteFreqHigh = NoteFreq<442>;      ///< 高音高 A4=442Hz

/**
 * @brief 音符时值定义（根据BPM计算）
 * @tparam Bpm 每分钟节拍数（Beats Per Minute）
 * @note  所有时值以四分音符为基准计算，单位为毫秒(ms)
 *        计算公式：四分音符时值 = 60000ms / BPM
 *
 * @example
 * ```cpp
 * // 使用BPM=120的音符时值
 * using Duration = NoteDuration<120>;
 * BuzzerNote note(note_freq::kC5, Duration::kQuarter);
 *
 * // 使用BPM=80的音符时值（较慢）
 * using SlowDuration = NoteDuration<80>;
 * BuzzerNote slow_note(note_freq::kC5, SlowDuration::kQuarter);
 * ```
 */
template <u16 Bpm>
struct NoteDuration {
  // 四分音符时值 = 60000ms / BPM
  static constexpr u16 kQuarter = 60000 / Bpm;

  // 基本音符时值
  static constexpr u16 kWhole = kQuarter * 4;         ///< 全音符
  static constexpr u16 kHalf = kQuarter * 2;          ///< 二分音符
  static constexpr u16 kEighth = kQuarter / 2;        ///< 八分音符
  static constexpr u16 kSixteenth = kQuarter / 4;     ///< 十六分音符
  static constexpr u16 kThirtySecond = kQuarter / 8;  ///< 三十二分音符

  // 附点音符时值（原时值的1.5倍）
  static constexpr u16 kDottedWhole = kWhole + kHalf;         ///< 附点全音符
  static constexpr u16 kDottedHalf = kHalf + kQuarter;        ///< 附点二分音符
  static constexpr u16 kDottedQuarter = kQuarter + kEighth;   ///< 附点四分音符
  static constexpr u16 kDottedEighth = kEighth + kSixteenth;  ///< 附点八分音符

  // 三连音时值（原时值的2/3）
  static constexpr u16 kTripletHalf = (kHalf * 2) / 3;        ///< 三连音二分音符
  static constexpr u16 kTripletQuarter = (kQuarter * 2) / 3;  ///< 三连音四分音符
  static constexpr u16 kTripletEighth = (kEighth * 2) / 3;    ///< 三连音八分音符
};

// 常用BPM的别名
using NoteDuration60 = NoteDuration<60>;    ///< 慢速 (60 BPM)
using NoteDuration80 = NoteDuration<80>;    ///< 稍慢 (80 BPM)
using NoteDuration100 = NoteDuration<100>;  ///< 中速 (100 BPM)
using NoteDuration120 = NoteDuration<120>;  ///< 标准速度 (120 BPM)
using NoteDuration140 = NoteDuration<140>;  ///< 稍快 (140 BPM)
using NoteDuration160 = NoteDuration<160>;  ///< 快速 (160 BPM)

/**
 * @brief 蜂鸣器音乐序列的基类，继承自SequenceGenerator
 */
class BuzzerMelody : public SequenceGenerator<BuzzerNote> {
 public:
  using Note = BuzzerNote;
};

/**
 * @brief 一些预定义的蜂鸣器音乐/提示音
 * @note  这些音乐都是按1ms一个step来设计的，如果音乐的播放速度听起来不对（太快/太慢），检查一下Update调用的频率是否正确
 */
namespace buzzer_melody {

/**
 * @brief 静音
 */
class Silent : public BuzzerMelody {
 public:
  Silent() = default;

  BuzzerNote Update() override { return BuzzerNote(0, 0); }

  void Reset() override {}
};

/**
 * @brief 滴滴响几声
 */
template <usize BeepsCount>
class Beeps : public BuzzerMelody {
 public:
  Beeps() = default;

  BuzzerNote Update() override {
    using Freq = NoteFreqStandard;
    using Duration = NoteDuration140;

    if (note_index_ >= BeepsCount * 2) {
      return BuzzerNote(0, 0);  // 播放完毕
    }

    BuzzerNote note;
    if (note_index_ % 2 == 0) {
      note = BuzzerNote(Freq::kC6, Duration::kSixteenth);  // 响声
    } else {
      note = BuzzerNote(Freq::kRest, Duration::kSixteenth);  // 静音
    }

    elapsed_time_ += 1;  // 假设1ms调用一次
    if (elapsed_time_ >= note.duration) {
      note_index_++;
      elapsed_time_ = 0;
    }

    return note;
  }

  void Reset() override {
    note_index_ = 0;
    elapsed_time_ = 0;
  }

 private:
  usize note_index_{0};
  u32 elapsed_time_{0};
};

/**
 * @brief 启动提示音
 */
class Startup : public BuzzerMelody {
 public:
  Startup() = default;

  BuzzerNote Update() override {
    using Freq = NoteFreqStandard;
    using Duration = NoteDuration160;

    constexpr std::array<BuzzerNote, 3> kMelody = {
        BuzzerNote(Freq::kC6, Duration::kSixteenth),
        BuzzerNote(Freq::kE6, Duration::kSixteenth),
        BuzzerNote(Freq::kG6, Duration::kEighth),
    };

    if (note_index_ >= kMelody.size()) {
      return BuzzerNote(0, 0);  // 播放完毕
    }

    auto note = kMelody[note_index_];

    // 更新计时器
    elapsed_time_ += 1;  // 假设1ms调用一次
    if (elapsed_time_ >= note.duration) {
      note_index_++;
      elapsed_time_ = 0;
    }

    return note;
  }

  void Reset() override {
    note_index_ = 0;
    elapsed_time_ = 0;
  }

 private:
  usize note_index_{0};
  u32 elapsed_time_{0};
};

/**
 * @brief 错误提示音
 */
class Error : public BuzzerMelody {
 public:
  Error() = default;

  BuzzerNote Update() override {
    using Freq = NoteFreqStandard;
    using Duration = NoteDuration160;

    constexpr std::array<BuzzerNote, 4> kMelody = {
        BuzzerNote(Freq::kG5, Duration::kSixteenth),
        BuzzerNote(Freq::kRest, Duration::kSixteenth),
        BuzzerNote(Freq::kG5, Duration::kEighth),
        BuzzerNote(Freq::kRest, Duration::kSixteenth),
    };

    if (note_index_ >= kMelody.size()) {
      return BuzzerNote(0, 0);
    }

    auto note = kMelody[note_index_];

    elapsed_time_ += 1;
    if (elapsed_time_ >= note.duration) {
      note_index_++;
      elapsed_time_ = 0;
    }

    return note;
  }

  void Reset() override {
    note_index_ = 0;
    elapsed_time_ = 0;
  }

 private:
  usize note_index_{0};
  u32 elapsed_time_{0};
};

/**
 * @brief 成功提示音
 */
class Success : public BuzzerMelody {
 public:
  Success() = default;

  BuzzerNote Update() override {
    using Freq = NoteFreqStandard;
    using Duration = NoteDuration160;

    constexpr std::array<BuzzerNote, 2> kMelody = {
        BuzzerNote(Freq::kC6, Duration::kSixteenth),
        BuzzerNote(Freq::kC7, Duration::kQuarter),
    };

    if (note_index_ >= kMelody.size()) {
      return BuzzerNote(0, 0);
    }

    auto note = kMelody[note_index_];

    elapsed_time_ += 1;
    if (elapsed_time_ >= note.duration) {
      note_index_++;
      elapsed_time_ = 0;
    }

    return note;
  }

  void Reset() override {
    note_index_ = 0;
    elapsed_time_ = 0;
  }

 private:
  usize note_index_{0};
  u32 elapsed_time_{0};
};

/**
 * @brief 超级马里奥主题曲（片段）
 */
class SuperMario : public BuzzerMelody {
 public:
  SuperMario() = default;

  BuzzerNote Update() override {
    using Freq = NoteFreqStandard;
    using Duration = NoteDuration120;

    constexpr std::array kMelody = {
        BuzzerNote(Freq::kE6, Duration::kThirtySecond), BuzzerNote(Freq::kRest, Duration::kThirtySecond),
        BuzzerNote(Freq::kE6, Duration::kSixteenth),    BuzzerNote(Freq::kRest, Duration::kSixteenth),
        BuzzerNote(Freq::kE6, Duration::kSixteenth),    BuzzerNote(Freq::kRest, Duration::kSixteenth),
        BuzzerNote(Freq::kC6, Duration::kSixteenth),    BuzzerNote(Freq::kE6, Duration::kSixteenth),
        BuzzerNote(Freq::kRest, Duration::kSixteenth),  BuzzerNote(Freq::kG6, Duration::kEighth),
        BuzzerNote(Freq::kRest, Duration::kEighth),     BuzzerNote(Freq::kG5, Duration::kEighth),
        BuzzerNote(Freq::kRest, Duration::kEighth),     BuzzerNote(Freq::kC6, Duration::kSixteenth),
        BuzzerNote(Freq::kRest, Duration::kSixteenth),  BuzzerNote(Freq::kG5, Duration::kSixteenth),
    };

    if (note_index_ >= kMelody.size()) {
      return BuzzerNote(0, 0);
    }

    auto note = kMelody[note_index_];

    elapsed_time_ += 1;
    if (elapsed_time_ >= note.duration) {
      note_index_++;
      elapsed_time_ = 0;
    }

    return note;
  }

  void Reset() override {
    note_index_ = 0;
    elapsed_time_ = 0;
  }

 private:
  usize note_index_{0};
  u32 elapsed_time_{0};
};

/**
* @brief  "The LICK" - https://en.wikipedia.org/wiki/The_Lick
*
     88
     ""

     88 ,adPPYYba, 888888888 888888888
     88 ""     `Y8      a8P"      a8P"    for your Soul
     88 ,adPPPPP88   ,d8P'     ,d8P'
     88 88,    ,88 ,d8"      ,d8"                   This is my music...
     88 `"8bbdP"Y8 888888888 888888888
    ,88
  888P"
 */
class TheLick : public BuzzerMelody {
 public:
  TheLick() = default;

  BuzzerNote Update() override {
    using Freq = NoteFreqStandard;
    using Duration = NoteDuration<85>;

    constexpr std::array kMelody = {
        BuzzerNote(Freq::kD6, Duration::kSixteenth),      BuzzerNote(Freq::kE6, Duration::kSixteenth),
        BuzzerNote(Freq::kF6, Duration::kSixteenth),      BuzzerNote(Freq::kG6, Duration::kSixteenth),
        BuzzerNote(Freq::kDs6, Duration::kThirtySecond),  BuzzerNote(Freq::kE6, Duration::kSixteenth),
        BuzzerNote(Freq::kRest, Duration::kThirtySecond), BuzzerNote(Freq::kC6, Duration::kSixteenth),
        BuzzerNote(Freq::kD6, Duration::kDottedEighth),
    };

    if (note_index_ >= kMelody.size()) {
      return BuzzerNote(0, 0);  // 播放完毕
    }

    auto note = kMelody[note_index_];

    elapsed_time_ += 1;
    if (elapsed_time_ >= note.duration) {
      note_index_++;
      elapsed_time_ = 0;
    }

    return note;
  }

  void Reset() override {
    note_index_ = 0;
    elapsed_time_ = 0;
  }

 private:
  usize note_index_{0};
  u32 elapsed_time_{0};
};

// NOTE: 你可以参照上面的几个Melody，通过继承BuzzerMelody类来实现自己的音乐序列

}  // namespace buzzer_melody

/**
 * @brief 蜂鸣器控制器，功能为产生各种音乐/提示音，基于通用的SequencePlayer实现
 * @tparam MelodyTypes... 音乐旋律类型列表，所有类型必须继承自BuzzerMelody
 *
 * @example
 * ```cpp
 * // 创建蜂鸣器控制器，包含多种音乐/提示音
 * BuzzerController<
 *     buzzer_melody::Silent,
 *     buzzer_melody::Beeps<3>,
 *     buzzer_melody::Startup,
 *     buzzer_melody::Error,
 *     buzzer_melody::Success,
 *     buzzer_melody::SuperMarioLoop,
 *     buzzer_melody::TheLick
 * > buzzer_controller;
 *
 * // 播放启动提示音
 * buzzer_controller.Play<buzzer_melody::StartupBeep>();
 *
 * // 在1ms定时器中更新
 * auto note = buzzer_controller.Update();
 * if (note.frequency > 0) {
 *     SetBuzzerFrequency(note.frequency);
 * } else {
 *     StopBuzzer();
 * }
 * ```
 */
template <typename... MelodyTypes>
class BuzzerController : public SequencePlayer<BuzzerNote, MelodyTypes...> {
 public:
  using Base = SequencePlayer<BuzzerNote, MelodyTypes...>;
  using Note = BuzzerNote;

  /**
   * @brief           播放指定的音乐/提示音
   * @tparam Melody   要播放的旋律类型
   */
  template <typename Melody>
  void Play() {
    Base::template SetSequence<Melody>();
  }

  /**
   * @brief  停止播放（切换到静音）
   * @note   需要在MelodyTypes中包含一个返回静音的序列类型
   */
  void Stop() { Base::Reset(); }

  /**
   * @brief  获取当前音符（Update方法继承自SequencePlayer）
   * @return 当前音符数据
   */
  const Note &current_note() const { return Base::current_output(); }

  /**
   * @brief  检查当前是否正在发声
   * @return 如果频率大于0则返回true
   */
  bool is_playing() const { return Base::current_output().frequency > 0; }
};

}  // namespace rm::modules

#endif  // LIBRM_MODULES_BUZZER_CONTROLLER_HPP
