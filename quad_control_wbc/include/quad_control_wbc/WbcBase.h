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

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>

#include "quad_control_mpc/gait/MotionPhaseDefinition.h"
#include "quad_control_wbc/Task.h"


namespace quad_control {
using namespace ocs2;
using namespace quad_robot;

// Decision Variables: x = [\dot u^T, F^T, \tau^T]^T
class WbcBase {
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

 public:
  WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

  virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                          scalar_t period);

 protected:
  void updateMeasured(const vector_t& rbdStateMeasured);
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

  size_t getNumDecisionVars() const { return numDecisionVars_; }

  Task formulateFloatingBaseEomTask();
  Task formulateTorqueLimitsTask();
  Task formulateNoContactMotionTask();
  Task formulateFrictionConeTask();
  Task formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);
  Task formulateSwingLegTask();
  Task formulateContactForceTask(const vector_t& inputDesired) const;

  size_t numDecisionVars_;
  PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  CentroidalModelInfo info_;

  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  CentroidalModelPinocchioMapping mapping_;

  vector_t qMeasured_, vMeasured_, inputLast_;
  matrix_t j_, dj_;
  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // Task Parameters:
  vector_t torqueLimits_;
  scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
};

}  // namespace quad_control
