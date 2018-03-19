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

/* Author: Ioan Sucan */

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/ompl_interface/detail/constrained_valid_state_sampler.h>
#include <moveit/profiler/profiler.h>
#include <fstream>
#include "../../../../../../usr/include/yaml-cpp/node/node.h"

ompl_interface::OMPLInterface::OMPLInterface(const robot_model::RobotModelConstPtr& kmodel/*, const ros::NodeHandle& nh*/)
  : /*nh_(nh)
  , */kmodel_(kmodel)
  , constraint_sampler_manager_(new constraint_samplers::ConstraintSamplerManager())
  , context_manager_(kmodel, constraint_sampler_manager_)
  , constraints_library_(new ConstraintsLibrary(context_manager_))
  , use_constraints_approximations_(true)
  , simplify_solutions_(true)
{
  CONSOLE_BRIDGE_logInform("Initializing OMPL interface using ROS parameters");
  loadPlannerConfigurations();
  loadConstraintApproximations();
  loadConstraintSamplers();
}

ompl_interface::OMPLInterface::OMPLInterface(const robot_model::RobotModelConstPtr& kmodel,
                                             const planning_interface::PlannerConfigurationMap& pconfig/*,
                                             const ros::NodeHandle& nh*/)
  : /*nh_(nh)
  , */kmodel_(kmodel)
  , constraint_sampler_manager_(new constraint_samplers::ConstraintSamplerManager())
  , context_manager_(kmodel, constraint_sampler_manager_)
  , constraints_library_(new ConstraintsLibrary(context_manager_))
  , use_constraints_approximations_(true)
  , simplify_solutions_(true)
{
  CONSOLE_BRIDGE_logInform("Initializing OMPL interface using specified configuration");
  setPlannerConfigurations(pconfig);
  loadConstraintApproximations();
  loadConstraintSamplers();
}

ompl_interface::OMPLInterface::~OMPLInterface()
{
}

void ompl_interface::OMPLInterface::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig)
{
  planning_interface::PlannerConfigurationMap pconfig2 = pconfig;

  // construct default configurations for planning groups that don't have configs already passed in
  const std::vector<const robot_model::JointModelGroup*>& groups = kmodel_->getJointModelGroups();
  for (std::size_t i = 0; i < groups.size(); ++i)
  {
    if (pconfig.find(groups[i]->getName()) == pconfig.end())
    {
      planning_interface::PlannerConfigurationSettings empty;
      empty.name = empty.group = groups[i]->getName();
      pconfig2[empty.name] = empty;
    }
  }

  context_manager_.setPlannerConfigurations(pconfig2);
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::OMPLInterface::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req) const
{
  moveit_msgs::MoveItErrorCodes dummy;
  return getPlanningContext(planning_scene, req, dummy);
}

ompl_interface::ModelBasedPlanningContextPtr ompl_interface::OMPLInterface::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
  ModelBasedPlanningContextPtr ctx = context_manager_.getPlanningContext(planning_scene, req, error_code);
  if (ctx)
    configureContext(ctx);
  return ctx;
}

ompl_interface::ModelBasedPlanningContextPtr
ompl_interface::OMPLInterface::getPlanningContext(const std::string& config, const std::string& factory_type) const
{
  ModelBasedPlanningContextPtr ctx = context_manager_.getPlanningContext(config, factory_type);
  if (ctx)
    configureContext(ctx);
  return ctx;
}

void ompl_interface::OMPLInterface::configureContext(const ModelBasedPlanningContextPtr& context) const
{
  if (use_constraints_approximations_)
    context->setConstraintsApproximations(constraints_library_);
  else
    context->setConstraintsApproximations(ConstraintsLibraryPtr());
  context->simplifySolutions(simplify_solutions_);
}

void ompl_interface::OMPLInterface::loadConstraintApproximations(const std::string& path)
{
  constraints_library_->loadConstraintApproximations(path);
  std::stringstream ss;
  constraints_library_->printConstraintApproximations(ss);
  CONSOLE_BRIDGE_logInform("%s", ss.str().c_str());
}

void ompl_interface::OMPLInterface::saveConstraintApproximations(const std::string& path)
{
  constraints_library_->saveConstraintApproximations(path);
}

bool ompl_interface::OMPLInterface::saveConstraintApproximations()
{
  std::string cpath;// TODO
  //if (nh_.getParam("constraint_approximations_path", cpath))
  if(1)
  {
    saveConstraintApproximations(cpath);
    return true;
  }
  CONSOLE_BRIDGE_logWarn("ROS param 'constraint_approximations' not found. Unable to save constraint approximations");
  return false;
}

bool ompl_interface::OMPLInterface::loadConstraintApproximations()
{
  std::string cpath;
  //if (nh_.getParam("constraint_approximations_path", cpath))
  if(1) // TODO
  {
    loadConstraintApproximations(cpath);
    return true;
  }
  return false;
}

void ompl_interface::OMPLInterface::loadConstraintSamplers()
{
  constraint_sampler_manager_loader_.reset(
      new constraint_sampler_manager_loader::ConstraintSamplerManagerLoader(constraint_sampler_manager_));
}

bool ompl_interface::OMPLInterface::loadPlannerConfiguration(
    const std::string& group_name, const std::string& planner_id,
    const std::map<std::string, std::string>& group_params, const YAML::Node& node,
    planning_interface::PlannerConfigurationSettings& planner_config)
{

  planner_config.name = group_name + "[" + planner_id + "]";
  planner_config.group = group_name;

  // default to specified parameters of the group (overridden by configuration specific parameters)
  planner_config.config = group_params;

  // read parameters specific for this configuration
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
    planner_config.config[it->first.as<std::string>()] = it->second.as<std::string>();
    //std::cout << it->first.as<std::string>() << ":" << it->second.as<std::string>() << '\n';
  }

  return true;

}

void ompl_interface::OMPLInterface::loadPlannerConfigurations()
{
  const std::vector<std::string>& group_names = kmodel_->getJointModelGroupNames();
  planning_interface::PlannerConfigurationMap pconfig;

  // read the planning configuration for each group
  pconfig.clear();

  YAML::Node config = YAML::LoadFile("ompl_planning.yaml");
  for (std::size_t i = 0; i < group_names.size(); ++i)
  {
    // the set of planning parameters that can be specific for the group (inherited by configurations of that group)
    static const std::string KNOWN_GROUP_PARAMS[] = { "projection_evaluator", "longest_valid_segment_fraction",
                                                      "enforce_joint_model_state_space" };

    // get parameters specific for the robot planning group
    std::map<std::string, std::string> specific_group_params;

    for (size_t k = 0; k < sizeof(KNOWN_GROUP_PARAMS) / sizeof(std::string); ++k) {
      if (config[group_names[i]][KNOWN_GROUP_PARAMS[k]])
      {
        specific_group_params[KNOWN_GROUP_PARAMS[k]] = config[group_names[i]][KNOWN_GROUP_PARAMS[k]].as<std::string>();
        //std::cout << specific_group_params[KNOWN_GROUP_PARAMS[k]] << '\n';
      }
    }

    // add default planner configuration
    planning_interface::PlannerConfigurationSettings default_pc;
    std::string default_planner_id;

    if (default_planner_id.empty())
    {
      default_pc.group = group_names[i];
      default_pc.config = specific_group_params;
      default_pc.config["type"] = "geometric::RRTConnect";
    }
    default_pc.name = group_names[i];  // this is the name of the default config
    pconfig[default_pc.name] = default_pc;

    // get parameters specific to each planner type
    YAML::Node planner = config[group_names[i]]["planner_configs"];
    for (size_t j = 0; j < planner.size(); ++j) {
      std::string planner_id = planner[j].as<std::string>();
      //std::cout << planner_id << '\n';

      planning_interface::PlannerConfigurationSettings pc;
      YAML::Node planner_param = config["planner_configs"][planner_id];
      if (loadPlannerConfiguration(group_names[i], planner_id, specific_group_params, planner_param, pc))
        pconfig[pc.name] = pc;
      }

    }

  for (planning_interface::PlannerConfigurationMap::iterator it = pconfig.begin(); it != pconfig.end(); ++it)
  {
    CONSOLE_BRIDGE_logDebug("parameters", "Parameters for configuration '%s'", it->first.c_str() );
    for (std::map<std::string, std::string>::const_iterator config_it = it->second.config.begin();
         config_it != it->second.config.end(); ++config_it)
      CONSOLE_BRIDGE_logDebug("parameters - %s = %s", config_it->first.c_str(), config_it->second.c_str());
  }
  setPlannerConfigurations(pconfig);
}

void ompl_interface::OMPLInterface::printStatus()
{
  CONSOLE_BRIDGE_logInform("OMPL ROS interface is running.");
}
