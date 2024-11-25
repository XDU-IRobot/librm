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
 * @file  librm/modules/algorithm/pid.cc
 * @brief PID控制器
 */

#include "pid.h"

#include <cstring>

#include "librm/modules/algorithm/utils.hpp"

namespace rm::modules::algorithm {

using utils::absConstrain;
using utils::LoopConstrain;

PID::PID(PIDType type, f32 kp, f32 ki, f32 kd, f32 max_out, f32 max_iout)
    : kp_(kp), ki_(ki), kd_(kd), type_(type), max_out_(max_out), max_iout_(max_iout) {}

void PID::Update(f32 set, f32 ref) {
  this->error_[2] = this->error_[1];
  this->error_[1] = this->error_[0];

  this->set_ = set;
  this->fdb_ = ref;
  this->error_[0] = set - ref;

  switch (this->type_) {
    case PIDType::kPosition:
      this->p_out_ = this->kp_ * this->error_[0];
      this->i_out_ += this->ki_ * this->error_[0];

      // update derivative term
      this->d_buf_[2] = this->d_buf_[1];
      this->d_buf_[1] = this->d_buf_[0];
      this->d_buf_[0] = this->error_[0] - this->error_[1];

      this->d_out_ = this->kd_ * this->d_buf_[0];
      this->i_out_ = absConstrain(this->i_out_, this->max_iout_);
      this->out_ = this->p_out_ + this->i_out_ + this->d_out_;
      this->out_ = absConstrain(this->out_, this->max_out_);
      break;

    case PIDType::kDelta:
      this->p_out_ = this->kp_ * (this->error_[0] - this->error_[1]);
      this->i_out_ = this->ki_ * this->error_[0];

      this->d_buf_[2] = this->d_buf_[1];
      this->d_buf_[1] = this->d_buf_[0];
      this->d_buf_[0] = this->error_[0] - 2.0f * this->error_[1] + this->error_[2];

      this->d_out_ = this->kd_ * this->d_buf_[0];
      this->out_ += this->p_out_ + this->i_out_ + this->d_out_;
      this->out_ = absConstrain(this->out_, this->max_out_);
      break;
  }
}

void PID::Update(f32 set, f32 ref, f32 external_diff) {
  this->error_[2] = this->error_[1];
  this->error_[1] = this->error_[0];

  this->set_ = set;
  this->fdb_ = ref;
  this->error_[0] = set - ref;

  switch (this->type_) {
    case PIDType::kPosition:
      this->p_out_ = this->kp_ * this->error_[0];
      this->i_out_ += this->ki_ * this->error_[0];

      // update derivative term
      this->d_buf_[2] = this->d_buf_[1];
      this->d_buf_[1] = this->d_buf_[0];
      this->d_buf_[0] = external_diff;

      this->d_out_ = this->kd_ * this->d_buf_[0];
      this->i_out_ = absConstrain(this->i_out_, this->max_iout_);
      this->out_ = this->p_out_ + this->i_out_ + this->d_out_;
      this->out_ = absConstrain(this->out_, this->max_out_);
      break;

    case PIDType::kDelta:
      this->p_out_ = this->kp_ * (this->error_[0] - this->error_[1]);
      this->i_out_ = this->ki_ * this->error_[0];

      this->d_buf_[2] = this->d_buf_[1];
      this->d_buf_[1] = this->d_buf_[0];
      this->d_buf_[0] = external_diff;

      this->d_out_ = this->kd_ * this->d_buf_[0];
      this->out_ += this->p_out_ + this->i_out_ + this->d_out_;
      this->out_ = absConstrain(this->out_, this->max_out_);
      break;
  }
}

void PID::Clear() {
  this->set_ = 0;
  this->fdb_ = 0;
  this->out_ = 0;
  this->p_out_ = 0;
  this->i_out_ = 0;
  this->d_out_ = 0;
  memset(this->d_buf_, 0, 3);
  memset(this->error_, 0, 3);
}

f32 PID::value() const { return this->out_; }

RingPID::RingPID(PIDType type, f32 kp, f32 ki, f32 kd, f32 max_out, f32 max_iout, f32 cycle)
    : PID(type, kp, ki, kd, max_out, max_iout), cycle_(cycle) {}

void RingPID::Update(f32 set, f32 ref) {
  this->error_[2] = this->error_[1];
  this->error_[1] = this->error_[0];

  this->set_ = set;
  this->fdb_ = ref;
  this->error_[0] = set - ref;

  this->error_[0] = LoopConstrain(this->error_[0], -this->cycle_ / 2, this->cycle_ / 2);

  switch (this->type_) {
    case PIDType::kPosition:
      this->p_out_ = this->kp_ * this->error_[0];
      this->i_out_ += this->ki_ * this->error_[0];

      // update derivative term
      this->d_buf_[2] = this->d_buf_[1];
      this->d_buf_[1] = this->d_buf_[0];
      this->d_buf_[0] = this->error_[0] - this->error_[1];

      this->d_out_ = this->kd_ * this->d_buf_[0];
      this->i_out_ = absConstrain(this->i_out_, this->max_iout_);
      this->out_ = this->p_out_ + this->i_out_ + this->d_out_;
      this->out_ = absConstrain(this->out_, this->max_out_);
      break;

    case PIDType::kDelta:
      this->p_out_ = this->kp_ * (this->error_[0] - this->error_[1]);
      this->i_out_ = this->ki_ * this->error_[0];

      this->d_buf_[2] = this->d_buf_[1];
      this->d_buf_[1] = this->d_buf_[0];
      this->d_buf_[0] = this->error_[0] - 2.0f * this->error_[1] + this->error_[2];

      this->d_out_ = this->kd_ * this->d_buf_[0];
      this->out_ += this->p_out_ + this->i_out_ + this->d_out_;
      this->out_ = absConstrain(this->out_, this->max_out_);
      break;
  }
}

void RingPID::Update(f32 set, f32 ref, f32 external_diff) {
  this->error_[2] = this->error_[1];
  this->error_[1] = this->error_[0];

  this->set_ = set;
  this->fdb_ = ref;
  this->error_[0] = set - ref;

  this->error_[0] = LoopConstrain(this->error_[0], -this->cycle_ / 2, this->cycle_ / 2);

  switch (this->type_) {
    case PIDType::kPosition:
      this->p_out_ = this->kp_ * this->error_[0];
      this->i_out_ += this->ki_ * this->error_[0];

      // update derivative term
      this->d_buf_[2] = this->d_buf_[1];
      this->d_buf_[1] = this->d_buf_[0];
      this->d_buf_[0] = external_diff;

      this->d_out_ = this->kd_ * this->d_buf_[0];
      this->i_out_ = absConstrain(this->i_out_, this->max_iout_);
      this->out_ = this->p_out_ + this->i_out_ + this->d_out_;
      this->out_ = absConstrain(this->out_, this->max_out_);
      break;

    case PIDType::kDelta:
      this->p_out_ = this->kp_ * (this->error_[0] - this->error_[1]);
      this->i_out_ = this->ki_ * this->error_[0];

      this->d_buf_[2] = this->d_buf_[1];
      this->d_buf_[1] = this->d_buf_[0];
      this->d_buf_[0] = external_diff;

      this->d_out_ = this->kd_ * this->d_buf_[0];
      this->out_ += this->p_out_ + this->i_out_ + this->d_out_;
      this->out_ = absConstrain(this->out_, this->max_out_);
      break;
  }
}

}  // namespace rm::modules::algorithm
