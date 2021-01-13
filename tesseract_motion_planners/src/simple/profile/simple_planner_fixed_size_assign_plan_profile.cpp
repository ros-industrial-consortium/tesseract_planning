/**
 * @file simple_planner_default_plan_profile.cpp
 * @brief
 *
 * @author Matthew Powelson
 * @date July 23, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>
#include <tesseract_motion_planners/simple/step_generators/fixed_size_assign_position.h>

namespace tesseract_planning
{
SimplePlannerFixedSizeAssignPlanProfile::SimplePlannerFixedSizeAssignPlanProfile(int freespace_steps, int linear_steps)
  : freespace_steps_(freespace_steps), linear_steps_(linear_steps)
{
  apply();
}

void SimplePlannerFixedSizeAssignPlanProfile::apply()
{
  state_generator = [this](const PlanInstruction& prev,
                           const PlanInstruction& base,
                           const PlannerRequest& request,
                           const ManipulatorInfo& manip_info) {
    return simplePlannerGeneratorFixedSizeAssign(
        prev, base, request, manip_info, this->freespace_steps_, this->linear_steps_);
  };
}

int SimplePlannerFixedSizeAssignPlanProfile::getFreespaceSteps() { return freespace_steps_; }
void SimplePlannerFixedSizeAssignPlanProfile::setFreespaceSteps(int freespace_steps)
{
  freespace_steps_ = freespace_steps;
  apply();
}

int SimplePlannerFixedSizeAssignPlanProfile::getLinearSteps() { return linear_steps_; }
void SimplePlannerFixedSizeAssignPlanProfile::setLinearSteps(int linear_steps)
{
  linear_steps_ = linear_steps;
  apply();
}
}  // namespace tesseract_planning