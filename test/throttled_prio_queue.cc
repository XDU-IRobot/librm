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

#include <gtest/gtest.h>

#include "librm.hpp"

using namespace rm::modules;

struct TestPacket {
  uint32_t id;
};

// 测试限流优先级队列的基本功能，包括优先级调度、过期丢弃以及 std::optional 返回值
TEST(ThrottledPrioQueueTest, ThrottlingAndOptionalReturn) {
  ThrottledPrioQueue<TestPacket, 10> queue(10);  // 10ms 间隔
  uint32_t now = 1000;

  queue.Push({1}, 10, now + 100);
  queue.Push({2}, 5, now + 100);

  // 第一次调用：应获取 ID 1 (高优先级)
  auto msg1 = queue.Process(now);
  ASSERT_TRUE(msg1.has_value());
  EXPECT_EQ(msg1->id, 1);

  // 立即第二次调用：未到 10ms 间隔，应返回 nullopt
  auto msg2 = queue.Process(now + 5);
  EXPECT_FALSE(msg2.has_value());

  // 时间推进后调用：应获取 ID 2
  auto msg3 = queue.Process(now + 11);
  ASSERT_TRUE(msg3.has_value());
  EXPECT_EQ(msg3->id, 2);
}

// 测试过期消息在 std::optional 中被正确丢弃
TEST(ThrottledPrioQueueTest, DeadlineDiscardInOptional) {
  ThrottledPrioQueue<TestPacket, 10> queue(0);  // 0ms 间隔方便测试过期
  uint32_t now = 1000;

  queue.Push({99}, 10, now + 50);  // 50ms 后过期

  // 模拟时间流逝到过期后
  auto msg = queue.Process(now + 60);
  EXPECT_FALSE(msg.has_value());
  EXPECT_TRUE(queue.empty());
}