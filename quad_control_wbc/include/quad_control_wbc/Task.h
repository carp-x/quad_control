/******************************************************************************
Copyright (c) 2026, Yuxin Li. All rights reserved.
Copyright (c) 2022, QiayuanLiao. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#pragma once

#include <utility>

#include <ocs2_core/Types.h>


namespace quad_control {
using namespace ocs2;

class Task {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Task() = default;

  Task(matrix_t a, vector_t b, matrix_t d, vector_t f) : a_(std::move(a)), d_(std::move(d)), b_(std::move(b)), f_(std::move(f)) {}

  explicit Task(size_t numDecisionVars)
      : Task(matrix_t::Zero(0, numDecisionVars), vector_t::Zero(0), matrix_t::Zero(0, numDecisionVars), vector_t::Zero(0)) {}

  Task operator+(const Task& rhs) const {
    return {concatenateMatrices(a_, rhs.a_), concatenateVectors(b_, rhs.b_), concatenateMatrices(d_, rhs.d_),
            concatenateVectors(f_, rhs.f_)};
  }

  Task operator*(scalar_t rhs) const {  // clang-format off
    return {a_.cols() > 0 ? rhs * a_ : a_,
            b_.cols() > 0 ? rhs * b_ : b_,
            d_.cols() > 0 ? rhs * d_ : d_,
            f_.cols() > 0 ? rhs * f_ : f_};  // clang-format on
  }

  matrix_t a_, d_;
  vector_t b_, f_;

  static matrix_t concatenateMatrices(matrix_t m1, matrix_t m2) {
    if (m1.cols() <= 0) {
      return m2;
    } else if (m2.cols() <= 0) {
      return m1;
    }
    assert(m1.cols() == m2.cols());
    matrix_t res(m1.rows() + m2.rows(), m1.cols());
    res << m1, m2;
    return res;
  }

  static vector_t concatenateVectors(const vector_t& v1, const vector_t& v2) {
    if (v1.cols() <= 0) {
      return v2;
    } else if (v2.cols() <= 0) {
      return v1;
    }
    assert(v1.cols() == v2.cols());
    vector_t res(v1.rows() + v2.rows());
    res << v1, v2;
    return res;
  }
};

}  // namespace quad_control
