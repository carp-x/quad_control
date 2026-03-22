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

#include <utility>
#include <qpOASES.hpp>

#include "quad_control_wbc/HierarchicalQp.h"


namespace quad_control {

HierarchicalQp::HierarchicalQp(Task task, HierarchicalQp::HierarchicalQpPtr higherProblem) : task_(std::move(task)), higherProblem_(std::move(higherProblem)) {
  initVars();
  formulateProblem();
  solveProblem();
  buildZMatrix();
  stackSlackSolutions();
}

void HierarchicalQp::initVars() {
  numSlackVars_ = task_.D().rows();
  hasEqConstraints_ = task_.A().rows() > 0;
  hasIneqConstraints_ = numSlackVars_ > 0;

  if (higherProblem_ != nullptr) {
    stackedTasksPrev_ = higherProblem_->getStackedTasks();
    xPrev_ = higherProblem_->getSolutions();
    stackedZPrev_ = higherProblem_->getStackedZMatrix();
    stackedSlackSolutionsPrev_ = higherProblem_->getStackedSlackSolutions();

    numDecisionVars_ = stackedZPrev_.cols();
    numPrevSlackVars_ = higherProblem_->getSlackedNumVars();
  }
  else {
    numDecisionVars_ = std::max(task_.A().cols(), task_.D().cols());
    numPrevSlackVars_ = 0;

    stackedTasksPrev_ = Task(numDecisionVars_);
    stackedZPrev_ = matrix_t::Identity(numDecisionVars_, numDecisionVars_);
    stackedSlackSolutionsPrev_ = Eigen::VectorXd::Zero(0);
    xPrev_ = Eigen::VectorXd::Zero(numDecisionVars_);
  }

  stackedTasks_ = task_ + stackedTasksPrev_;

  eyeNvNv_ = matrix_t::Identity(numSlackVars_, numSlackVars_);
  zeroNvNx_ = matrix_t::Zero(numSlackVars_, numDecisionVars_);
}

void HierarchicalQp::formulateProblem() {
  buildHMatrix();
  buildCVector();
  buildDMatrix();
  buildFVector();
}

void HierarchicalQp::solveProblem() {
  auto qpProblem = qpOASES::QProblem(numDecisionVars_ + numSlackVars_, f_.size());
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  qpProblem.setOptions(options);
  int nWsr = 20;

  qpProblem.init(h_.data(), c_.data(), d_.data(), nullptr, nullptr, nullptr, f_.data(), nWsr);
  vector_t qpSol(numDecisionVars_ + numSlackVars_);

  qpProblem.getPrimalSolution(qpSol.data());

  decisionVarsSolutions_ = qpSol.head(numDecisionVars_);
  slackVarsSolutions_ = qpSol.tail(numSlackVars_);
}

void HierarchicalQp::buildZMatrix() {
  if (hasEqConstraints_) {
    assert((task_.A().cols() > 0));
    stackedZ_ = stackedZPrev_ * (task_.A() * stackedZPrev_).fullPivLu().kernel();
  } 
  else {
    stackedZ_ = stackedZPrev_;
  }
}

void HierarchicalQp::stackSlackSolutions() {
  if (higherProblem_ != nullptr) {
    stackedSlackVars_ = Task::concatenateVectors(higherProblem_->getStackedSlackSolutions(), slackVarsSolutions_);
  } 
  else {
    stackedSlackVars_ = slackVarsSolutions_;
  }
}

void HierarchicalQp::buildHMatrix() {
  matrix_t zTaTaz(numDecisionVars_, numDecisionVars_);

  if (hasEqConstraints_) {
    matrix_t aCurrZPrev = task_.A() * stackedZPrev_;
    zTaTaz = aCurrZPrev.transpose() * aCurrZPrev + 1e-12 * matrix_t::Identity(numDecisionVars_, numDecisionVars_);
  } 
  else {
    zTaTaz.setZero();
  }

  h_ = (matrix_t(numDecisionVars_ + numSlackVars_, numDecisionVars_ + numSlackVars_)
            << zTaTaz, zeroNvNx_.transpose(),
               zeroNvNx_, eyeNvNv_)
           .finished();
}

void HierarchicalQp::buildCVector() {
  vector_t c = vector_t::Zero(numDecisionVars_ + numSlackVars_);
  vector_t zeroVec = vector_t::Zero(numSlackVars_);

  vector_t temp(numDecisionVars_);
  if (hasEqConstraints_) {
    temp = (task_.A() * stackedZPrev_).transpose() * (task_.A() * xPrev_ - task_.b());
  } 
  else {
    temp.setZero();
  }

  c_ = (vector_t(numDecisionVars_ + numSlackVars_) << temp, zeroVec).finished();
}

void HierarchicalQp::buildDMatrix() {
  matrix_t stackedZero = matrix_t::Zero(numPrevSlackVars_, numSlackVars_);

  matrix_t dCurrZ;
  if (hasIneqConstraints_) {
    dCurrZ = task_.D() * stackedZPrev_;
  } 
  else {
    dCurrZ = matrix_t::Zero(0, numDecisionVars_);
  }

  d_ = (matrix_t(2 * numSlackVars_ + numPrevSlackVars_, numDecisionVars_ + numSlackVars_)
            << zeroNvNx_, -eyeNvNv_,
                stackedTasksPrev_.D() * stackedZPrev_, stackedZero,
                dCurrZ, -eyeNvNv_)
           .finished();
}

void HierarchicalQp::buildFVector() {
  vector_t zeroVec = vector_t::Zero(numSlackVars_);

  vector_t fMinusDXPrev;
  if (hasIneqConstraints_) {
    fMinusDXPrev = task_.f() - task_.D() * xPrev_;
  } 
  else {
    fMinusDXPrev = vector_t::Zero(0);
  }

  f_ = (vector_t(2 * numSlackVars_ + numPrevSlackVars_) << zeroVec,
        stackedTasksPrev_.f() - stackedTasksPrev_.D() * xPrev_ + stackedSlackSolutionsPrev_, fMinusDXPrev)
           .finished();
}

}  // namespace quad_control
