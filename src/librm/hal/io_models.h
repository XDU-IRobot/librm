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

/**
 * @file    librm/hal/io_models.h
 * @brief   定义阻塞IO、异步IO、信号IO三种IO模型，用于HAL层针对各种不同通信方式的外设设计API
 */

#ifndef LIBRM_HAL_IO_MODEL_H
#define LIBRM_HAL_IO_MODEL_H

#include <functional>

#include "librm/core/typedefs.h"

namespace rm::hal {

/**
 * @brief 阻塞读接口
 */
class BlockIInterface {
 public:
  virtual ~BlockIInterface() = default;

  virtual void Read(void* data, usize size) = 0;
};

/**
 * @brief 阻塞写接口
 */
class BlockOInterface {
 public:
  virtual ~BlockOInterface() = default;

  virtual void Write(const void* data, usize size) = 0;
};

/**
 * @brief 阻塞读写接口
 */
class BlockIOInterface : public BlockIInterface, public BlockOInterface {};

/**
 * @brief 异步读接口
 */
template <typename CallbackType>
class AsyncIInterface {
 public:
  virtual ~AsyncIInterface() = default;

  virtual void Read(void* data, usize size, std::function<CallbackType> callback) = 0;
};

/**
 * @brief 异步写接口
 */
template <typename CallbackType>
class AsyncOInterface {
 public:
  virtual ~AsyncOInterface() = default;

  virtual void Write(const void* data, usize size, std::function<CallbackType> callback) = 0;
};

/**
 * @brief 异步读写接口
 */
template <typename ICallbackType, typename OCallbackType>
class AsyncIOInterface : public AsyncIInterface<ICallbackType>, public AsyncOInterface<OCallbackType> {};

/**
 * @brief 信号型读接口
 */
template <typename... CallbackArgs>
class SignalIInterface {
 public:
  virtual ~SignalIInterface() = default;

  virtual void CallbackOnRead(CallbackArgs... args) = 0;
};

/**
 *  @defgroup io_model_type_traits IO模型类型特征定义
 *  @{
 */

template <typename T>
struct is_async_io : std::false_type {};

template <typename CallbackType>
struct is_async_io<AsyncIInterface<CallbackType>> : std::true_type {};

template <typename CallbackType>
struct is_async_io<AsyncOInterface<CallbackType>> : std::true_type {};

template <typename ICallbackType, typename OCallbackType>
struct is_async_io<AsyncIOInterface<ICallbackType, OCallbackType>> : std::true_type {};

template <typename... CallbackArgs>
struct is_async_io<SignalIInterface<CallbackArgs...>> : std::true_type {};

template <typename T>
inline constexpr bool is_async_io_v = is_async_io<T>::value;

/** @} */

}  // namespace rm::hal

#endif  // LIBRM_HAL_IO_MODEL_H
