/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string/join.hpp>
#include <sstream>

const std::string planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC = "display_planned_path";
const std::string planning_pipeline::PlanningPipeline::MOTION_PLAN_REQUEST_TOPIC = "motion_plan_request";
const std::string planning_pipeline::PlanningPipeline::MOTION_CONTACTS_TOPIC = "display_contacts";

planning_pipeline::PlanningPipeline::PlanningPipeline(const robot_model::RobotModelConstPtr& model,
                                                      /* const ros::NodeHandle& nh,*/
                                                      const std::string& planner_plugin_param_name,
                                                      const std::string& adapter_plugins_param_name)
  :/* nh_(nh), */kmodel_(model)
{

  planner_plugin_name_ = planner_plugin_param_name;

  std::string adapters = adapter_plugins_param_name;

    boost::char_separator<char> sep(" ");
    boost::tokenizer<boost::char_separator<char> > tok(adapters, sep);
    for (boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin(); beg != tok.end(); ++beg)
      adapter_plugin_names_.push_back(*beg);


  configure();
}

planning_pipeline::PlanningPipeline::PlanningPipeline(const robot_model::RobotModelConstPtr& model,
                                                     /* const ros::NodeHandle& nh, */const std::string&
        planner_plugin_name,
                                                      const std::vector<std::string>& adapter_plugin_names)
  : /*nh_(nh), */planner_plugin_name_(planner_plugin_name), adapter_plugin_names_(adapter_plugin_names), kmodel_(model)
{
  configure();
}

void planning_pipeline::PlanningPipeline::configure()
{
  check_solution_paths_ = false;  // this is set to true below
  publish_received_requests_ = false;
  display_computed_motion_plans_ = false;  // this is set to true below

  // load the planning plugin
  try
  {
    planner_plugin_loader_.reset(new class_loader::MultiLibraryClassLoader(true));
    planner_plugin_loader_->loadLibrary("libmoveit_ompl_planner_plugin.so");
  }
  catch (class_loader::ClassLoaderException & ex)
  {
    CONSOLE_BRIDGE_logError("ClassLoaderException: %s", ex.what());
  }
  /*
  std::vector<std::string> classes;
  if (planner_plugin_loader_)
    classes = planner_plugin_loader_->getDeclaredClasses();
  if (planner_plugin_name_.empty() && classes.size() == 1)
  {
    planner_plugin_name_ = classes[0];
    ROS_INFO("No '~planning_plugin' parameter specified, but only '%s' planning plugin is available. Using that one.",
             planner_plugin_name_.c_str());
  }
  if (planner_plugin_name_.empty() && classes.size() > 1)
  {
    planner_plugin_name_ = classes[0];
    ROS_INFO("Multiple planning plugins available. You should specify the '~planning_plugin' parameter. Using '%s' for "
             "now.",
             planner_plugin_name_.c_str());
  }
  */
  try
  {
    planner_instance_.reset(planner_plugin_loader_->createUnmanagedInstance<planning_interface::PlannerManager>
            ("ompl_interface::OMPLPlannerManager"));

    if (!planner_instance_->initialize(kmodel_, ""))
      throw std::runtime_error("Unable to initialize planning plugin");
    CONSOLE_BRIDGE_logInform("Using planning interface ' %s '" , planner_instance_->getDescription().c_str());
  }
  catch (class_loader::ClassLoaderException& ex)
  {
    CONSOLE_BRIDGE_logError("Exception while loading planner ' %s ' : %s",
                     planner_plugin_name_ .c_str(),  ex.what());
  }

  // load the planner request adapters
  if (!adapter_plugin_names_.empty())
  {
    std::vector<planning_request_adapter::PlanningRequestAdapterConstPtr> ads;
    try
    {
      planner_plugin_loader_.reset(new class_loader::MultiLibraryClassLoader(true));
      planner_plugin_loader_->loadLibrary("libmoveit_default_planning_request_adapter_plugins.so");
    }
    catch (class_loader::ClassLoaderException& ex)
    {
      CONSOLE_BRIDGE_logError("Exception while creating planning plugin loader %s", ex.what());
    }

    if (planner_plugin_loader_)
      for (std::size_t i = 0; i < adapter_plugin_names_.size(); ++i)
      {
        planning_request_adapter::PlanningRequestAdapterConstPtr ad;
        try
        {
          ad.reset(planner_plugin_loader_->createUnmanagedInstance<planning_request_adapter::PlanningRequestAdapter>(adapter_plugin_names_[i]));
        }
        catch (class_loader::ClassLoaderException& ex)
        {
          CONSOLE_BRIDGE_logError("Exception while loading planning adapter plugin '%s': %s",
                                  adapter_plugin_names_[i].c_str(), ex.what());
        }
        if (ad)
          ads.push_back(ad);
      }
    if (!ads.empty())
    {
      adapter_chain_.reset(new planning_request_adapter::PlanningRequestAdapterChain());
      for (std::size_t i = 0; i < ads.size(); ++i)
      {
        CONSOLE_BRIDGE_logInform("Using planning request adapter '%s'", ads[i]->getDescription().c_str());
        adapter_chain_->addAdapter(ads[i]);
      }
    }
  }
  displayComputedMotionPlans(true);
  checkSolutionPaths(true);
}

void planning_pipeline::PlanningPipeline::displayComputedMotionPlans(bool flag)
{
#if 0
  if (display_computed_motion_plans_ && !flag)
    display_path_publisher_.shutdown();
  else if (!display_computed_motion_plans_ && flag)
    display_path_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>(DISPLAY_PATH_TOPIC, 10, true);
  display_computed_motion_plans_ = flag;
#endif
}

void planning_pipeline::PlanningPipeline::publishReceivedRequests(bool flag)
{
#if 0
  if (publish_received_requests_ && !flag)
    received_request_publisher_.shutdown();
  else if (!publish_received_requests_ && flag)
    received_request_publisher_ = nh_.advertise<moveit_msgs::MotionPlanRequest>(MOTION_PLAN_REQUEST_TOPIC, 10, true);
  publish_received_requests_ = flag;
#endif
}

void planning_pipeline::PlanningPipeline::checkSolutionPaths(bool flag)
{
#if 0
  if (check_solution_paths_ && !flag)
    contacts_publisher_.shutdown();
  else if (!check_solution_paths_ && flag)
    contacts_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(MOTION_CONTACTS_TOPIC, 100, true);
  check_solution_paths_ = flag;
#endif
}

bool planning_pipeline::PlanningPipeline::generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                       const planning_interface::MotionPlanRequest& req,
                                                       planning_interface::MotionPlanResponse& res) const
{
  std::vector<std::size_t> dummy;
  return generatePlan(planning_scene, req, res, dummy);
}

bool planning_pipeline::PlanningPipeline::generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                       const planning_interface::MotionPlanRequest& req,
                                                       planning_interface::MotionPlanResponse& res,
                                                       std::vector<std::size_t>& adapter_added_state_index) const
{
  // broadcast the request we are about to work on, if needed

  /*if (publish_received_requests_)
    received_request_publisher_.publish(req);*/
  adapter_added_state_index.clear();

  if (!planner_instance_)
  {
    CONSOLE_BRIDGE_logError("No planning plugin loaded. Cannot plan.");
    return false;
  }

  bool solved = false;
  try
  {
    if (adapter_chain_)
    {
      solved = adapter_chain_->adaptAndPlan(planner_instance_, planning_scene, req, res, adapter_added_state_index);
      if (!adapter_added_state_index.empty())
      {
        std::stringstream ss;
        for (std::size_t i = 0; i < adapter_added_state_index.size(); ++i)
          ss << adapter_added_state_index[i] << " ";
        CONSOLE_BRIDGE_logInform("Planning adapters have added states at index positions: [ %s]", ss.str().c_str());
      }
    }
    else
    {
      planning_interface::PlanningContextPtr context =
          planner_instance_->getPlanningContext(planning_scene, req, res.error_code_);
      solved = context ? context->solve(res) : false;
    }
  }
  catch (std::exception& ex)
  {
    CONSOLE_BRIDGE_logError("Exception caught: '%s'", ex.what());
    return false;
  }
  bool valid = true;

  if (solved && res.trajectory_)
  {
    std::size_t state_count = res.trajectory_->getWayPointCount();
    CONSOLE_BRIDGE_logDebug("Motion planner reported a solution path with %d states", state_count);
    if (check_solution_paths_)
    {
      std::vector<std::size_t> index;
      if (!planning_scene->isPathValid(*res.trajectory_, req.path_constraints, req.group_name, false, &index))
      {
        // check to see if there is any problem with the states that are found to be invalid
        // they are considered ok if they were added by a planning request adapter
        bool problem = false;
        for (std::size_t i = 0; i < index.size() && !problem; ++i)
        {
          bool found = false;
          for (std::size_t j = 0; j < adapter_added_state_index.size(); ++j)
            if (index[i] == adapter_added_state_index[j])
            {
              found = true;
              break;
            }
          if (!found)
            problem = true;
        }
        if (problem)
        {
          if (index.size() == 1 && index[0] == 0)  // ignore cases when the robot starts at invalid location
            CONSOLE_BRIDGE_logDebug("It appears the robot is starting at an invalid state, but that is ok.");
          else
          {
            valid = false;
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;

            // display error messages
            std::stringstream ss;
            for (std::size_t i = 0; i < index.size(); ++i)
              ss << index[i] << " ";
            CONSOLE_BRIDGE_logError("Computed path is not valid. Invalid states at index locations: [ ");

            // call validity checks in verbose mode for the problematic states
            visualization_msgs::MarkerArray arr;
            for (std::size_t i = 0; i < index.size(); ++i)
            {
              // check validity with verbose on
              const robot_state::RobotState& kstate = res.trajectory_->getWayPoint(index[i]);
              planning_scene->isStateValid(kstate, req.path_constraints, req.group_name, true);

              // compute the contacts if any
              collision_detection::CollisionRequest c_req;
              collision_detection::CollisionResult c_res;
              c_req.contacts = true;
              c_req.max_contacts = 10;
              c_req.max_contacts_per_pair = 3;
              c_req.verbose = false;
              planning_scene->checkCollision(c_req, c_res, kstate);
              if (c_res.contact_count > 0)
              {
                visualization_msgs::MarkerArray arr_i;
                collision_detection::getCollisionMarkersFromContacts(arr_i, planning_scene->getPlanningFrame(),
                                                                     c_res.contacts);
                arr.markers.insert(arr.markers.end(), arr_i.markers.begin(), arr_i.markers.end());
              }
            }
            CONSOLE_BRIDGE_logError("Completed listing of explanations for invalid states.");
            /*if (!arr.markers.empty())
              contacts_publisher_.publish(arr);*/
          }
        }
        else
          CONSOLE_BRIDGE_logDebug("Planned path was found to be valid, except for states that were added by planning "
                                      "request "
                    "adapters, but that is ok.");
      }
      else
        CONSOLE_BRIDGE_logDebug("Planned path was found to be valid when rechecked");
    }
  }

  // display solution path if needed
  if (display_computed_motion_plans_ && solved)
  {
    moveit_msgs::DisplayTrajectory disp;
    disp.model_id = kmodel_->getName();
    disp.trajectory.resize(1);
    res.trajectory_->getRobotTrajectoryMsg(disp.trajectory[0]);
    robot_state::robotStateToRobotStateMsg(res.trajectory_->getFirstWayPoint(), disp.trajectory_start);
    //display_path_publisher_.publish(disp);
  }

  return solved && valid;
}

void planning_pipeline::PlanningPipeline::terminate() const
{
  if (planner_instance_)
    planner_instance_->terminate();
}
