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

#include <gtest/gtest.h>

#include "quad_control_wbc/HierarchicalQp.h"

using namespace quad_control;

TEST(HQP, twoTask) {
  srand(0);
  Task task0{matrix_t::Random(2, 4),
             vector_t::Ones(2),  
             matrix_t::Random(2, 4),
             vector_t::Ones(2)};
  Task task1{matrix_t::Ones(2, 4),
             task0.b(),  
             task0.D(),
             task0.f()};

  std::shared_ptr<HierarchicalQp> hQp0 = std::make_shared<HierarchicalQp>(task0);
  std::shared_ptr<HierarchicalQp> hQp1 = std::make_shared<HierarchicalQp>(task1, hQp0);

  vector_t x0 = hQp0->getSolutions(), x_1 = hQp1->getSolutions();
  vector_t slack0 = hQp0->getStackedSlackSolutions(), slack_1 = hQp1->getStackedSlackSolutions();

  std::cout << x0.transpose() << std::endl;
  std::cout << x_1.transpose() << std::endl;
  std::cout << slack0.transpose() << std::endl;
  std::cout << slack_1.transpose() << std::endl;

  scalar_t prec = 1e-6;

  if (slack0.isApprox(vector_t::Zero(slack0.size()))) EXPECT_TRUE((task0.A() * x0).isApprox(task0.b(), prec));
  if (slack_1.isApprox(vector_t::Zero(slack_1.size()))) {
    EXPECT_TRUE((task1.A() * x_1).isApprox(task1.b(), prec));
    EXPECT_TRUE((task0.A() * x_1).isApprox(task0.b(), prec));
  }

  vector_t y = task0.D() * x0;
  for (int i = 0; i < y.size(); ++i) EXPECT_TRUE(y[i] <= task0.f()[i] + slack0[i]);
  y = task1.D() * x_1;
  for (int i = 0; i < y.size(); ++i) EXPECT_TRUE(y[i] <= task1.f()[i] + slack_1[i]);
}
