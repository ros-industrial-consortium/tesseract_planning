/**
 * @file trajopt_ifopt_utils.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_utils.h>
#include <trajopt_ifopt/trajopt_ifopt.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_problem.h>
#include <tesseract_command_language/types.h>
#include <tesseract_common/utils.h>

#include <ifopt/problem.h>

namespace tesseract_planning
{
ifopt::ConstraintSet::Ptr createCartesianPositionConstraint(const Eigen::Isometry3d& target,
                                                            trajopt_ifopt::JointPosition::ConstPtr var,
                                                            trajopt_ifopt::KinematicsInfo::ConstPtr kin_info,
                                                            std::string source_link,
                                                            const Eigen::Isometry3d& source_tcp,
                                                            const Eigen::Ref<const Eigen::VectorXd>& coeffs)
{
  std::vector<int> indices;
  std::vector<double> constraint_coeffs;
  for (Eigen::Index i = 0; i < coeffs.rows(); ++i)
  {
    if (!tesseract_common::almostEqualRelativeAndAbs(coeffs(i), 0.0))
    {
      indices.push_back(static_cast<int>(i));
      constraint_coeffs.push_back(coeffs(i));
    }
  }

  trajopt_ifopt::CartPosInfo cart_info(
      kin_info,
      target,
      source_link,
      source_tcp,
      Eigen::Map<Eigen::VectorXi>(indices.data(), static_cast<Eigen::Index>(indices.size())));
  auto constraint = std::make_shared<trajopt_ifopt::CartPosConstraint>(cart_info, var, "CartPos_" + var->GetName());
  return constraint;
}

bool addCartesianPositionConstraint(std::shared_ptr<ifopt::Problem> nlp,
                                    const Eigen::Isometry3d& target,
                                    trajopt_ifopt::JointPosition::ConstPtr var,
                                    trajopt_ifopt::KinematicsInfo::ConstPtr kin_info,
                                    std::string source_link,
                                    const Eigen::Isometry3d& source_tcp,
                                    const Eigen::Ref<const Eigen::VectorXd>& coeffs)
{
  auto constraint = createCartesianPositionConstraint(target, var, kin_info, source_link, source_tcp, coeffs);
  nlp->AddConstraintSet(constraint);
  return true;
}

bool addCartesianPositionSquaredCost(std::shared_ptr<ifopt::Problem> nlp,
                                     const Eigen::Isometry3d& target,
                                     trajopt_ifopt::JointPosition::ConstPtr var,
                                     trajopt_ifopt::KinematicsInfo::ConstPtr kin_info,
                                     std::string source_link,
                                     const Eigen::Isometry3d& source_tcp,
                                     const Eigen::Ref<const Eigen::VectorXd>& coeffs)
{
  std::vector<double> constraint_coeffs;
  std::vector<double> cost_coeffs;
  for (Eigen::Index i = 0; i < coeffs.rows(); ++i)
  {
    if (tesseract_common::almostEqualRelativeAndAbs(coeffs(i), 0.0))
    {
      constraint_coeffs.push_back(0);
    }
    else
    {
      constraint_coeffs.push_back(1);
      cost_coeffs.push_back(coeffs(i));
    }
  }

  auto constraint = createCartesianPositionConstraint(
      target,
      var,
      kin_info,
      source_link,
      source_tcp,
      Eigen::Map<Eigen::VectorXd>(constraint_coeffs.data(), static_cast<Eigen::Index>(constraint_coeffs.size())));

  // Must link the variables to the constraint since that happens in AddConstraintSet
  constraint->LinkWithVariables(nlp->GetOptVariables());
  auto cost = std::make_shared<trajopt_ifopt::SquaredCost>(
      constraint, Eigen::Map<Eigen::VectorXd>(constraint_coeffs.data(), static_cast<Eigen::Index>(cost_coeffs.size())));
  nlp->AddCostSet(cost);
  return true;
}

bool addCartesianPositionAbsoluteCost(std::shared_ptr<ifopt::Problem> nlp,
                                      const Eigen::Isometry3d& target,
                                      trajopt_ifopt::JointPosition::ConstPtr var,
                                      trajopt_ifopt::KinematicsInfo::ConstPtr kin_info,
                                      std::string source_link,
                                      const Eigen::Isometry3d& source_tcp,
                                      const Eigen::Ref<const Eigen::VectorXd>& coeffs)
{
  std::vector<double> constraint_coeffs;
  std::vector<double> cost_coeffs;
  for (Eigen::Index i = 0; i < coeffs.rows(); ++i)
  {
    if (tesseract_common::almostEqualRelativeAndAbs(coeffs(i), 0.0))
    {
      constraint_coeffs.push_back(0);
    }
    else
    {
      constraint_coeffs.push_back(1);
      cost_coeffs.push_back(coeffs(i));
    }
  }

  auto constraint = createCartesianPositionConstraint(
      target,
      var,
      kin_info,
      source_link,
      source_tcp,
      Eigen::Map<Eigen::VectorXd>(constraint_coeffs.data(), static_cast<Eigen::Index>(constraint_coeffs.size())));

  // Must link the variables to the constraint since that happens in AddConstraintSet
  constraint->LinkWithVariables(nlp->GetOptVariables());
  auto cost = std::make_shared<trajopt_ifopt::AbsoluteCost>(
      constraint, Eigen::Map<Eigen::VectorXd>(constraint_coeffs.data(), static_cast<Eigen::Index>(cost_coeffs.size())));
  nlp->AddCostSet(cost);
  return true;
}

ifopt::ConstraintSet::Ptr createJointPositionConstraint(const JointWaypoint& joint_waypoint,
                                                        trajopt_ifopt::JointPosition::ConstPtr var,
                                                        const Eigen::VectorXd& /*coeffs*/)
{
  assert(var);
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars(1, var);

  ifopt::ConstraintSet::Ptr constraint;
  if (!joint_waypoint.isToleranced())
  {
    constraint = std::make_shared<trajopt_ifopt::JointPosConstraint>(
        joint_waypoint.waypoint, vars, "JointPos_" + var->GetName());
  }
  else
  {
    Eigen::VectorXd lower_limit = joint_waypoint.waypoint + joint_waypoint.lower_tolerance;
    Eigen::VectorXd upper_limit = joint_waypoint.waypoint + joint_waypoint.upper_tolerance;
    auto bounds = trajopt_ifopt::toBounds(lower_limit, upper_limit);
    constraint = std::make_shared<trajopt_ifopt::JointPosConstraint>(bounds, vars, "JointPos_" + var->GetName());
  }

  return constraint;
}

bool addJointPositionConstraint(std::shared_ptr<ifopt::Problem> nlp,
                                const JointWaypoint& joint_waypoint,
                                trajopt_ifopt::JointPosition::ConstPtr var,
                                const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  auto constraint = createJointPositionConstraint(joint_waypoint, var, coeff);
  nlp->AddConstraintSet(constraint);
  return true;
}

bool addJointPositionSquaredCost(std::shared_ptr<ifopt::Problem> nlp,
                                 const JointWaypoint& joint_waypoint,
                                 trajopt_ifopt::JointPosition::ConstPtr var,
                                 const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  auto vel_constraint = createJointPositionConstraint(joint_waypoint, var, coeff);

  // Must link the variables to the constraint since that happens in AddConstraintSet
  vel_constraint->LinkWithVariables(nlp->GetOptVariables());
  auto vel_cost = std::make_shared<trajopt_ifopt::SquaredCost>(vel_constraint, coeff);
  nlp->AddCostSet(vel_cost);
  return true;
}

std::vector<ifopt::ConstraintSet::Ptr>
createCollisionConstraints(std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars,
                           const tesseract_environment::Environment::ConstPtr& env,
                           const ManipulatorInfo& manip_info,
                           const trajopt_ifopt::TrajOptCollisionConfig::ConstPtr& config,
                           const std::vector<int>& fixed_indices)
{
  std::vector<ifopt::ConstraintSet::Ptr> constraints;
  if (config->type == tesseract_collision::CollisionEvaluatorType::NONE)
    return constraints;

  // Add a collision cost for all steps
  auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(vars.size());
  if (config->type == tesseract_collision::CollisionEvaluatorType::DISCRETE)
  {
    for (std::size_t i = 0; i < vars.size(); ++i)
    {
      if (std::find(fixed_indices.begin(), fixed_indices.end(), i) != fixed_indices.end())
        continue;

      auto kin = env->getManipulatorManager()->getFwdKinematicSolver(manip_info.manipulator);
      auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
          env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

      auto collision_evaluator = std::make_shared<trajopt_ifopt::SingleTimestepCollisionEvaluator>(
          collision_cache, kin, env, adjacency_map, Eigen::Isometry3d::Identity(), config);

      constraints.push_back(std::make_shared<trajopt_ifopt::DiscreteCollisionConstraintIfopt>(
          collision_evaluator,
          trajopt_ifopt::DiscreteCombineCollisionData(trajopt_ifopt::CombineCollisionDataMethod::WEIGHTED_AVERAGE),
          vars[i]));
    }
  }
  else if (config->type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    for (std::size_t i = 0; i < vars.size(); ++i)
    {
      if (std::find(fixed_indices.begin(), fixed_indices.end(), i) != fixed_indices.end())
        continue;

      auto kin = env->getManipulatorManager()->getFwdKinematicSolver(manip_info.manipulator);
      auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
          env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

      auto collision_evaluator = std::make_shared<trajopt_ifopt::LVSDiscreteCollisionEvaluator>(
          collision_cache, kin, env, adjacency_map, Eigen::Isometry3d::Identity(), config);

      std::array<trajopt_ifopt::JointPosition::ConstPtr, 3> position_vars;
      if (i == 0)
        position_vars = { nullptr, vars[i], vars[i + 1] };
      else if (i == vars.size() - 1)
        position_vars = { vars[i - 1], vars[i], nullptr };
      else
        position_vars = { vars[i - 1], vars[i], vars[i + 1] };

      constraints.push_back(std::make_shared<trajopt_ifopt::ContinuousCollisionConstraintIfopt>(
          collision_evaluator,
          trajopt_ifopt::ContinuousCombineCollisionData(trajopt_ifopt::CombineCollisionDataMethod::WEIGHTED_AVERAGE),
          position_vars));
    }
  }
  else
  {
    for (std::size_t i = 0; i < vars.size(); ++i)
    {
      if (std::find(fixed_indices.begin(), fixed_indices.end(), i) != fixed_indices.end())
        continue;

      auto kin = env->getManipulatorManager()->getFwdKinematicSolver(manip_info.manipulator);
      auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
          env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

      auto collision_evaluator = std::make_shared<trajopt_ifopt::LVSContinuousCollisionEvaluator>(
          collision_cache, kin, env, adjacency_map, Eigen::Isometry3d::Identity(), config);

      std::array<trajopt_ifopt::JointPosition::ConstPtr, 3> position_vars;
      if (i == 0)
        position_vars = { nullptr, vars[i], vars[i + 1] };
      else if (i == vars.size() - 1)
        position_vars = { vars[i - 1], vars[i], nullptr };
      else
        position_vars = { vars[i - 1], vars[i], vars[i + 1] };

      constraints.push_back(std::make_shared<trajopt_ifopt::ContinuousCollisionConstraintIfopt>(
          collision_evaluator,
          trajopt_ifopt::ContinuousCombineCollisionData(trajopt_ifopt::CombineCollisionDataMethod::WEIGHTED_AVERAGE),
          position_vars));
    }
  }

  return constraints;
}

bool addCollisionConstraint(std::shared_ptr<ifopt::Problem> nlp,
                            std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars,
                            const tesseract_environment::Environment::ConstPtr& env,
                            const ManipulatorInfo& manip_info,
                            const trajopt_ifopt::TrajOptCollisionConfig::ConstPtr& config,
                            const std::vector<int>& fixed_indices)
{
  auto constraints = createCollisionConstraints(vars, env, manip_info, config, fixed_indices);
  for (auto& constraint : constraints)
    nlp->AddConstraintSet(constraint);
  return true;
}

bool addCollisionSquaredCost(std::shared_ptr<ifopt::Problem> nlp,
                             std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars,
                             const tesseract_environment::Environment::ConstPtr& env,
                             const ManipulatorInfo& manip_info,
                             const trajopt_ifopt::TrajOptCollisionConfig::ConstPtr& config,
                             const std::vector<int>& fixed_indices)
{
  // Coefficients are applied within the constraint
  auto constraints = createCollisionConstraints(vars, env, manip_info, config, fixed_indices);
  for (auto& constraint : constraints)
  {
    // Must link the variables to the constraint since that happens in AddConstraintSet
    constraint->LinkWithVariables(nlp->GetOptVariables());
    auto cost = std::make_shared<trajopt_ifopt::SquaredCost>(constraint);
    nlp->AddCostSet(cost);
  }
  return true;
}

ifopt::ConstraintSet::Ptr createJointVelocityConstraint(const Eigen::Ref<const Eigen::VectorXd>& target,
                                                        const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& vars,
                                                        const Eigen::VectorXd& /*coeffs*/)

{
  assert(!vars.empty());
  trajopt_ifopt::JointVelConstraint::Ptr vel_constraint =
      std::make_shared<trajopt_ifopt::JointVelConstraint>(target, vars, "JointVelocity");
  return vel_constraint;
}

bool addJointVelocityConstraint(std::shared_ptr<ifopt::Problem> nlp,
                                std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars,
                                const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  if (vars.empty())
    return true;

  Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(vars.front()->GetJointNames().size()));
  auto vel_constraint = createJointVelocityConstraint(vel_target, vars, coeff);
  nlp->AddConstraintSet(vel_constraint);
  return true;
}

bool addJointVelocitySquaredCost(std::shared_ptr<ifopt::Problem> nlp,
                                 std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars,
                                 const Eigen::Ref<const Eigen::VectorXd>& coeff)
{
  if (vars.empty())
    return true;

  Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(vars.front()->GetJointNames().size()));
  auto vel_constraint = createJointVelocityConstraint(vel_target, vars, coeff);

  // Must link the variables to the constraint since that happens in AddConstraintSet
  vel_constraint->LinkWithVariables(nlp->GetOptVariables());
  auto vel_cost = std::make_shared<trajopt_ifopt::SquaredCost>(vel_constraint);
  nlp->AddCostSet(vel_cost);
  return true;
}

}  // namespace tesseract_planning
