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

#include <utility>
#include <memory>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>

namespace ocs2 {
namespace quad_robot {

class LeggedSelfCollisionVisualization : public GeometryInterfaceVisualization {
 public:
  LeggedSelfCollisionVisualization(PinocchioInterface pinocchioInterface, 
                                   PinocchioGeometryInterface geometryInterface,
                                   const CentroidalModelPinocchioMapping& mapping,
                                   scalar_t maxUpdateFrequency = 50.0)
      : GeometryInterfaceVisualization(std::move(pinocchioInterface), std::move(geometryInterface), "odom"),
        mappingPtr_(mapping.clone()),
        lastTime_(std::numeric_limits<scalar_t>::lowest()),
        minPublishTimeDifference_(1.0 / maxUpdateFrequency) {}

  void update(const SystemObservation& observation) {
    if (observation.time - lastTime_ > minPublishTimeDifference_) {
      lastTime_ = observation.time;

      publishDistances(mappingPtr_->getPinocchioJointPosition(observation.state));
    }
  }

 private:
  std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr_;

  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;
};

}  // namespace quad_robot
}  // namespace ocs2
