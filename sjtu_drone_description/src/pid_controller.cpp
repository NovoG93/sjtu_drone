// Copyright 2023 Georg Novotny
//
// Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.gnu.org/licenses/gpl-3.0.en.html
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sjtu_drone_description/pid_controller.h"

PIDController::PIDController()
{

}

PIDController::~PIDController()
{

}


void PIDController::Load(sdf::ElementPtr _sdf, const std::string & prefix)
{
  gain_p = 5.0;
  gain_d = 1.0;
  gain_i = 0.0;
  time_constant = 0.0;
  limit = -1.0;

  if (!_sdf) {return;}
  if (_sdf->HasElement(prefix + "ProportionalGain")) {
    gain_p = _sdf->GetElement(prefix + "ProportionalGain")->Get<double>();
  }
  if (_sdf->HasElement(prefix + "DifferentialGain")) {
    gain_d = _sdf->GetElement(prefix + "DifferentialGain")->Get<double>();
  }
  if (_sdf->HasElement(prefix + "IntegralGain")) {
    gain_i = _sdf->GetElement(prefix + "IntegralGain")->Get<double>();
  }
  if (_sdf->HasElement(prefix + "TimeConstant")) {
    time_constant = _sdf->GetElement(prefix + "TimeConstant")->Get<double>();
  }
  if (_sdf->HasElement(prefix + "Limit")) {
    limit = _sdf->GetElement(prefix + "Limit")->Get<double>();
  }
}

double PIDController::update(double new_input, double x, double dx, double dt)
{
  // limit command
  if (limit > 0.0 && fabs(new_input) > limit) {new_input = (new_input < 0 ? -1.0 : 1.0) * limit;}

  // filter command
  if (dt + time_constant > 0.0) {
    input = (dt * new_input + time_constant * input) / (dt + time_constant);
    dinput = (new_input - input) / (dt + time_constant);
  }

  // update proportional, differential and integral errors
  p = input - x;
  d = dinput - dx;
  i = i + dt * p;

  // update control output
  output = gain_p * p + gain_d * d + gain_i * i;
  return output;
}

void PIDController::reset()
{
  input = dinput = 0;
  p = i = d = output = 0;
}
