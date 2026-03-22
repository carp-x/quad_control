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

#include "quad_control_wbc/HoQp.h"
#include "quad_control_wbc/HierarchicalWbc.h"


namespace quad_control {
vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period) {
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
  Task task1 = formulateBaseAccelTask(stateDesired, inputDesired, period) + formulateSwingLegTask();
  Task task2 = formulateContactForceTask(inputDesired);
  HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

  return hoQp.getSolutions();
}

}  // namespace quad_control
