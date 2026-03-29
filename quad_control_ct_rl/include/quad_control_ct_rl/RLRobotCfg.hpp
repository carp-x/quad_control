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

#pragma once

#include <ocs2_core/Types.h>

namespace quad_control {
using namespace ocs2;

struct RLRobotCfg {
  struct InitState {
    scalar_t LF_HAA_joint;
    scalar_t LF_HFE_joint;
    scalar_t LF_KFE_joint;

    scalar_t LH_HAA_joint;
    scalar_t LH_HFE_joint;
    scalar_t LH_KFE_joint;

    scalar_t RF_HAA_joint;
    scalar_t RF_HFE_joint;
    scalar_t RF_KFE_joint;

    scalar_t RH_HAA_joint;
    scalar_t RH_HFE_joint;
    scalar_t RH_KFE_joint;
  };

  struct ControlCfg {
    scalar_t stiffness;
    scalar_t damping;
    scalar_t action_scale;
    int decimation;
  };

  struct ObsScales {
    scalar_t lin_vel;
    scalar_t ang_vel;
    scalar_t dof_pos;
    scalar_t dof_vel;
  };

  InitState init_state;
  ControlCfg control_cfg;
  ObsScales obs_scales;

  scalar_t clip_actions;
  scalar_t clip_observations;
};

}  // namespace quad_control
