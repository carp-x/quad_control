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

#include <string>
#include <array>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/handle.hpp>


namespace quad_control {

struct JointHandle {
  std::string name;
  // Read
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> position;
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity;
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> effort;
  // Write
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> pos_des;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> vel_des;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> ff;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> kp;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> kd;
};

struct ImuHandle {
  std::string name;
  // Read
  std::array<std::reference_wrapper<const hardware_interface::LoanedStateInterface>, 4> ori;
  std::array<std::reference_wrapper<const hardware_interface::LoanedStateInterface>, 9> ori_cov;
  std::array<std::reference_wrapper<const hardware_interface::LoanedStateInterface>, 3> angular_vel;
  std::array<std::reference_wrapper<const hardware_interface::LoanedStateInterface>, 9> angular_vel_cov;
  std::array<std::reference_wrapper<const hardware_interface::LoanedStateInterface>, 3> linear_acc;
  std::array<std::reference_wrapper<const hardware_interface::LoanedStateInterface>, 9> linear_acc_cov;
};

struct ForceTorqueHandle {
  std::string name;
  // Read
  std::reference_wrapper<const hardware_interface::LoanedStateInterface> contact;

  bool incontact() const {
    return contact.get().get_optional<double>().value_or(0.0) > 0.99;
  }
};

}  // namespace quad_control
