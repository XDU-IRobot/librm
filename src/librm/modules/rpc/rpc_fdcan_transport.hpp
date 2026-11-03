/*
  Copyright (c) 2026 XDU-IRobot

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
 * @file  librm/modules/rpc/rpc_fdcan_transport.hpp
 * @brief 基于CAN FD的rpc_core传输层实现
 */

#ifndef LIBRM_MODULES_RPC_RPC_FDCAN_TRANSPORT_HPP
#define LIBRM_MODULES_RPC_RPC_FDCAN_TRANSPORT_HPP

#include <cstring>
#include <chrono>

#include <etl/random.h>
#include <etl/circular_buffer.h>
#include <etl/unordered_map.h>
#include <rpc_core/connection.hpp>

#include "librm/device/can_device.hpp"

namespace rm::modules {
/**
 * @brief 基于CAN FD实现的一种rpc_core框架的传输层
 */
class FdCanTransport : public rpc_core::connection {
  /**
   * @brief 基于CAN FD总线的大数据传输类
   *
   * 该类实现了一个基于CAN FD总线的大数据传输协议，支持将大数据包分片发送和接收。
   * 通过定义起始帧、数据分片帧和结束帧，实现了可靠的数据传输机制。
   */
  class Impl : public device::CanDevice {
    struct StdIdOffset {
      enum {
        kFrameStart = 0,
        kFrameEnd,
        kPayload,
        kHeartbeat = 40,  // 给一个很低的优先级，防止数据帧被同时发送的心跳仲裁掉
      };
    };

   public:
    static constexpr uint32_t kMaxPayloadBytes = 4096;
    static constexpr uint32_t kHeartbeatIntervalMs = 200;  ///< 心跳间隔，小于超时时间的一半

    Impl(hal::CanInterface &can, uint16_t rx_stdid_base, uint16_t tx_stdid_base)
        : CanDevice(can, rx_stdid_base + StdIdOffset::kFrameStart, rx_stdid_base + StdIdOffset::kPayload,
                    rx_stdid_base + StdIdOffset::kFrameEnd, rx_stdid_base + StdIdOffset::kHeartbeat),
          tx_buf_{},
          rx_state_{},
          rx_stdid_base_{rx_stdid_base},
          tx_stdid_base_{tx_stdid_base},
          last_tx_time_{std::chrono::steady_clock::now()} {}

    /**
     * @brief 发送一个大数据包，该函数会将其分片并通过CAN FD总线发送
     * @param data 要发送的数据指针
     * @param size 要发送的数据大小
     */
    void Send(const uint8_t *data, uint32_t size) {
      // 更新最后发送时间，数据帧本身就能维持连接
      last_tx_time_ = std::chrono::steady_clock::now();

      // 1. 生成一个随机的帧ID，用于标识本次传输的所有分片
      tx_buf_.frame_start.frame_id = static_cast<uint8_t>(rng_() & 0xff);

      // 2. 准备起始帧，包含元数据：帧ID、总载荷长度和CRC8校验值
      tx_buf_.frame_start.payload_len = size;
      tx_buf_.frame_start.payload_crc8 = modules::Crc8(data, size, modules::CRC8_INIT);
      // 发送起始帧
      can_->Write(tx_stdid_base_ + StdIdOffset::kFrameStart, reinterpret_cast<const uint8_t *>(&tx_buf_.frame_start),
                  sizeof(tx_buf_.frame_start));

      // 3. 循环发送数据分片
      uint16_t fragment_seq = 0;
      uint32_t bytes_sent = 0;
      while (bytes_sent < size) {
        tx_buf_.payload.frame_id = tx_buf_.frame_start.frame_id;
        tx_buf_.payload.fragment_seq = fragment_seq++;
        // 计算当前分片的数据大小
        uint8_t fragment_size =
            std::min(static_cast<uint32_t>(sizeof(tx_buf_.payload.fragment_data)), size - bytes_sent);
        tx_buf_.payload.fragment_len = fragment_size;
        std::memcpy(tx_buf_.payload.fragment_data, data + bytes_sent, fragment_size);
        // 发送数据分片，DLC固定为64
        can_->Write(tx_stdid_base_ + StdIdOffset::kPayload, reinterpret_cast<const uint8_t *>(&tx_buf_.payload), 64);
        bytes_sent += fragment_size;
      }

      // 4. 发送结束帧，标识数据传输完成
      tx_buf_.frame_end.frame_id = tx_buf_.frame_start.frame_id;
      can_->Write(tx_stdid_base_ + StdIdOffset::kFrameEnd, reinterpret_cast<const uint8_t *>(&tx_buf_.frame_end),
                  sizeof(tx_buf_.frame_end));
    }

    /**
     * @brief 智能心跳发送：仅在长时间无数据传输时发送心跳
     *
     * 当有数据传输时，数据帧本身就能维持连接状态，无需额外发送心跳。
     * 只有当距离上次发送超过kHeartbeatIntervalMs时，才发送心跳包。
     */
    void SendHeartbeatIfNeeded() {
      auto now = std::chrono::steady_clock::now();
      auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tx_time_).count();
      if (elapsed_ms >= kHeartbeatIntervalMs) {
        can_->Write(tx_stdid_base_ + StdIdOffset::kHeartbeat, nullptr, 0);
        last_tx_time_ = now;
      }
    }

    void OnFrameRecv(std::function<void(std::string &)> callback) { frame_recv_callback_ = std::move(callback); }

   private:
    void RxCallback(const hal::CanFrame *msg) override {
      // 收到任何帧都更新连接状态，因为任何帧都能证明对方在线
      ReportStatus(kOk);

      // 判断帧类型
      if (msg->std_id == rx_stdid_base_ + StdIdOffset::kHeartbeat) {
        // 心跳帧，已在上面更新状态
      } else if (msg->std_id == rx_stdid_base_ + StdIdOffset::kFrameStart) {
        // 1. 收到起始帧，开始一个新的数据帧接收会话
        const auto *frame_start = reinterpret_cast<const decltype(tx_buf_.frame_start) *>(msg->data.data());
        if (static_cast<usize>(msg->dlc) != sizeof(*frame_start)) {
          return;  // 帧长度错误，直接丢弃
        }
        if (frame_start->payload_len > kMaxPayloadBytes) {
          // 载荷超出单帧允许的最大长度，直接丢弃该会话
          rx_sessions_.erase(frame_start->frame_id);
          return;
        }
        // 检查会话池是否已满，如果满了且是新会话，则淘汰最不活跃的会话
        if (rx_sessions_.find(frame_start->frame_id) == rx_sessions_.end() && rx_sessions_.full()) {
          EvictLeastRecentlyUsedSession();
        }
        auto &session = rx_sessions_[frame_start->frame_id];
        session.payload_len = frame_start->payload_len;
        session.payload_crc8 = frame_start->payload_crc8;
        session.next_fragment_seq = 0;
        session.last_active_time = std::chrono::steady_clock::now();  // 更新活跃时间
        session.buffer.clear();
        session.buffer.reserve(frame_start->payload_len);
      } else if (msg->std_id == rx_stdid_base_ + StdIdOffset::kPayload) {
        // 2. 收到数据分片
        const auto *payload = reinterpret_cast<const decltype(tx_buf_.payload) *>(msg->data.data());
        auto session_it = rx_sessions_.find(payload->frame_id);
        if (session_it == rx_sessions_.end()) {
          return;  // 未找到对应会话，说明起始帧缺失
        }
        auto &session = session_it->second;
        // 校验分片序号确保顺序正确
        if (payload->fragment_seq != session.next_fragment_seq) {
          rx_sessions_.erase(session_it);  // 顺序错误，丢弃该会话
          return;
        }
        const uint32_t data_len = payload->fragment_len;
        if (session.buffer.size() + data_len > session.payload_len) {
          rx_sessions_.erase(session_it);  // 数据超长，丢弃该会话
          return;
        }
        session.buffer.insert(session.buffer.end(), payload->fragment_data, payload->fragment_data + data_len);
        session.next_fragment_seq++;
        session.last_active_time = std::chrono::steady_clock::now();  // 更新活跃时间

      } else if (msg->std_id == rx_stdid_base_ + StdIdOffset::kFrameEnd) {
        // 3. 收到结束帧
        const auto *frame_end = reinterpret_cast<const decltype(tx_buf_.frame_end) *>(msg->data.data());
        auto session_it = rx_sessions_.find(frame_end->frame_id);
        if (session_it == rx_sessions_.end()) {
          return;  // 没有对应的会话，忽略
        }
        auto &session = session_it->second;
        if (session.buffer.size() != session.payload_len) {
          rx_sessions_.erase(session_it);  // 长度不一致，放弃接收
          return;
        }
        // 4. 数据重组和CRC校验
        uint8_t calculated_crc = modules::Crc8(session.buffer.empty() ? nullptr : session.buffer.data(),
                                               session.buffer.size(), modules::CRC8_INIT);
        if (calculated_crc == session.payload_crc8) {
          // 5. CRC校验通过，调用回调函数处理完整的数据帧
          if (frame_recv_callback_) {
            std::string received_data(session.buffer.begin(), session.buffer.end());
            frame_recv_callback_(received_data);
          }
        }
        rx_sessions_.erase(session_it);
      }
    }

    struct {
      struct __attribute__((packed)) {
        uint8_t frame_id;      ///< 这一帧数据的ID，随机生成
        uint16_t payload_len;  ///< 这一帧数据的有效载荷长度，单位字节
        uint8_t payload_crc8;  ///< 这一帧数据的有效载荷的CRC8校验值
      } frame_start;
      struct __attribute__((packed)) {
        uint8_t frame_id;           ///< 这一帧数据的ID，随机生成
        uint16_t fragment_seq;      ///< 这一片段的序号，从0开始
        uint8_t fragment_len;       ///< 这一片段的数据长度
        uint8_t fragment_data[60];  ///< 这一片段的数据
      } payload;
      struct __attribute__((packed)) {
        uint8_t frame_id;  ///< 这一帧数据的ID，随机生成
      } frame_end;

      static_assert(sizeof(frame_start) == 4);
      static_assert(sizeof(payload) == 64);
      static_assert(sizeof(frame_end) == 1);
    } tx_buf_;

    struct {
      bool is_receiving = false;
      uint8_t frame_id;
      uint16_t payload_len;
      uint8_t payload_crc8;
      uint16_t received_len;
      uint16_t next_fragment_seq;
    } rx_state_;

    uint16_t rx_stdid_base_;
    uint16_t tx_stdid_base_;
    std::chrono::steady_clock::time_point last_tx_time_;  ///< 最后一次发送数据的时间戳，用于智能心跳
    std::function<void(std::string &)> frame_recv_callback_;  ///< 收到完整帧数据时的回调
    struct RxSession {
      uint16_t payload_len = 0;
      uint8_t payload_crc8 = 0;
      uint16_t next_fragment_seq = 0;
      std::chrono::steady_clock::time_point last_active_time;  ///< 最后活跃时间，用于LRU淘汰
      etl::vector<uint8_t, kMaxPayloadBytes> buffer;           ///< 4kb应该够了，不够再调
    };
    etl::unordered_map<uint8_t,  ///< frame_id
                       RxSession,
                       5>                ///< 大多数情况下不会有超过3个并发的帧重组会话
        rx_sessions_;                    ///< 并发管理多个帧重组会话
    etl::random_xorshift rng_{114514u};  ///< 用于生成随机的frame_id

    /**
     * @brief 淘汰最不活跃的会话（LRU策略）
     *
     * 当会话池满时，找出最长时间未更新的会话并删除，为新会话腾出空间。
     * 这样可以自动清理因丢包而永远无法完成的僵尸会话。
     */
    void EvictLeastRecentlyUsedSession() {
      if (rx_sessions_.empty()) return;

      auto oldest_it = rx_sessions_.begin();
      auto oldest_time = oldest_it->second.last_active_time;

      for (auto it = rx_sessions_.begin(); it != rx_sessions_.end(); ++it) {
        if (it->second.last_active_time < oldest_time) {
          oldest_time = it->second.last_active_time;
          oldest_it = it;
        }
      }

      rx_sessions_.erase(oldest_it);
    }
  } impl_;

 public:
  FdCanTransport(hal::CanInterface &can, uint16_t rx_stdid_base, uint16_t tx_stdid_base)
      : impl_{can, rx_stdid_base, tx_stdid_base} {
    send_package_impl = [this](const std::string &payload) {
      impl_.Send(reinterpret_cast<const uint8_t *>(payload.data()), payload.size());
    };
    impl_.OnFrameRecv([this](std::string &payload) {
      if (on_recv_package) {
        on_recv_package(payload);
      }
    });
  }

  virtual ~FdCanTransport() = default;

  /**
   * @brief 智能心跳维护：仅在必要时发送心跳包
   *
   * 建议周期性调用(如每200ms调用一次)。如果距离上次发送数据已超过心跳间隔，
   * 才会实际发送心跳包。这样在有数据传输时不会浪费带宽发送心跳。
   */
  void SendHeartbeat() { impl_.SendHeartbeatIfNeeded(); }

  bool Connected() { return impl_.online_status() == device::Device::kOk; }
};
}  // namespace rm::modules

#endif  // LIBRM_MODULES_RPC_RPC_FDCAN_TRANSPORT_HPP
