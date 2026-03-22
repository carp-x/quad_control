/******************************************************************************
Copyright (c) 2026, Yuxin Li. All rights reserved.
Copyright (c) 2022, Qiayuan Liao. All rights reserved.

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

#include <memory>

#include "quad_control_wbc/Task.h"


namespace quad_control {
// Hierarchical Optimization Quadratic Program
class HoQp {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using HoQpPtr = std::shared_ptr<HoQp>;

  explicit HoQp(const Task& task) : HoQp(task, nullptr){};

  HoQp(Task task, HoQpPtr higherProblem);

  matrix_t getStackedZMatrix() const { return stackedZ_; }

  Task getStackedTasks() const { return stackedTasks_; }

  vector_t getStackedSlackSolutions() const { return stackedSlackVars_; }

  vector_t getSolutions() const {
    vector_t x = xPrev_ + stackedZPrev_ * decisionVarsSolutions_;
    return x;
  }

  size_t getSlackedNumVars() const { return stackedTasks_.D().rows(); }

 private:
  void initVars();
  void formulateProblem();
  void solveProblem();

  void buildHMatrix();
  void buildCVector();
  void buildDMatrix();
  void buildFVector();

  void buildZMatrix();
  void stackSlackSolutions();

  Task task_, stackedTasksPrev_, stackedTasks_;
  HoQpPtr higherProblem_;

  bool hasEqConstraints_{}, hasIneqConstraints_{};
  size_t numSlackVars_{}, numDecisionVars_{};
  matrix_t stackedZPrev_, stackedZ_;
  vector_t stackedSlackSolutionsPrev_, xPrev_;
  size_t numPrevSlackVars_{};

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> h_, d_;
  vector_t c_, f_;
  vector_t stackedSlackVars_, slackVarsSolutions_, decisionVarsSolutions_;

  // Convenience matrices that are used multiple times
  matrix_t eyeNvNv_;
  matrix_t zeroNvNx_;
};

}  // namespace quad_control
