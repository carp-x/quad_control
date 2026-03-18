/******************************************************************************
Copyright (c) 2026, Yuxin Li. All rights reserved.

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

#include <gtest/gtest.h>

#include "quad_control_wbc/HoQp.h"

using namespace quad_control;

TEST(HoQP, twoTask) {
  srand(0);
  Task task0, task1;
  task0.a_ = matrix_t::Random(2, 4);
  task0.b_ = vector_t::Ones(2);
  task0.d_ = matrix_t::Random(2, 4);
  task0.f_ = vector_t::Ones(2);
  task1 = task0;
  task1.a_ = matrix_t::Ones(2, 4);
  std::shared_ptr<HoQp> hoQp0 = std::make_shared<HoQp>(task0);
  std::shared_ptr<HoQp> hoQp1 = std::make_shared<HoQp>(task1, hoQp0);

  vector_t x0 = hoQp0->getSolutions(), x_1 = hoQp1->getSolutions();
  vector_t slack0 = hoQp0->getStackedSlackSolutions(), slack_1 = hoQp1->getStackedSlackSolutions();

  std::cout << x0.transpose() << std::endl;
  std::cout << x_1.transpose() << std::endl;
  std::cout << slack0.transpose() << std::endl;
  std::cout << slack_1.transpose() << std::endl;

  scalar_t prec = 1e-6;

  if (slack0.isApprox(vector_t::Zero(slack0.size()))) EXPECT_TRUE((task0.a_ * x0).isApprox(task0.b_, prec));
  if (slack_1.isApprox(vector_t::Zero(slack_1.size()))) {
    EXPECT_TRUE((task1.a_ * x_1).isApprox(task1.b_, prec));
    EXPECT_TRUE((task0.a_ * x_1).isApprox(task0.b_, prec));
  }

  vector_t y = task0.d_ * x0;
  for (int i = 0; i < y.size(); ++i) EXPECT_TRUE(y[i] <= task0.f_[i] + slack0[i]);
  y = task1.d_ * x_1;
  for (int i = 0; i < y.size(); ++i) EXPECT_TRUE(y[i] <= task1.f_[i] + slack_1[i]);
}
