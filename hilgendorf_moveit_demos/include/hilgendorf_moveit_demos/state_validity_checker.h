/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Dave Coleman, Ioan Sucan */

#ifndef MOVEIT_OMPL_DETAIL_STATE_VALIDITY_CHECKER_
#define MOVEIT_OMPL_DETAIL_STATE_VALIDITY_CHECKER_

#include <moveit_ompl/detail/threadsafe_state_storage.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <ompl/base/StateValidityChecker.h>
#include <moveit_ompl/model_based_state_space.h>

namespace moveit_ompl
{
class ModelBasedPlanningContext;

/** @class StateValidityChecker
    @brief An interface for a OMPL state validity checker*/
class StateValidityChecker : public ompl::base::StateValidityChecker
{
public:
  StateValidityChecker(const std::string& group_name,
                       ompl::base::SpaceInformationPtr& si,
                       const moveit::core::RobotState& start_state,
                       const planning_scene::PlanningSceneConstPtr& planning_scene,
                       moveit_ompl::ModelBasedStateSpacePtr& mb_state_space);

  virtual bool isValid(const ompl::base::State *state) const
  {
    return isValid(state, verbose_);
  }

  virtual bool isValid(const ompl::base::State *state, double &dist) const
  {
    return isValid(state, dist, verbose_);
  }

  bool isValid(const ompl::base::State *state, bool verbose) const;
  bool isValid(const ompl::base::State *state, double &dist, bool verbose) const;

  virtual double cost(const ompl::base::State *state) const;
  virtual double clearance(const ompl::base::State *state) const;

  void setVerbose(bool flag);

protected:

  std::string group_name_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  moveit_ompl::ModelBasedStateSpacePtr mb_state_space_;

  ompl::base::SpaceInformationPtr si_;
  TSStateStorage tss_;
  collision_detection::CollisionRequest collision_request_simple_;
  collision_detection::CollisionRequest collision_request_with_distance_;
  collision_detection::CollisionRequest collision_request_simple_verbose_;
  collision_detection::CollisionRequest collision_request_with_distance_verbose_;

  collision_detection::CollisionRequest collision_request_with_cost_;
  bool verbose_;
};
}

#endif