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

/* Author: Sachin Chitta */

#include <pluginlib/class_loader.h>
#include <console_bridge/console.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <boost/scoped_ptr.hpp>

// Class_laoder test
#include <class_loader/class_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>


int main(int argc, char** argv)
{
#if 0
  console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
  const char LIBRARY_1[] = "libmoveit_ompl_planner_plugin.so";
  try {
    class_loader::ClassLoader loader1(LIBRARY_1, false);
    std::cout <<  loader1.createInstance<planning_interface::PlannerManager>("ompl_interface::OMPLPlannerManager")
            ->getDescription() << std::endl;
  } catch (class_loader::ClassLoaderException & e) {
    CONSOLE_BRIDGE_logError("ClassLoaderException: %s", e.what());
  }
#endif
#if 1

  // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using a planner is pretty easy. Planners are
  // setup as plugins in MoveIt! and you can use the ROS pluginlib
  // interface to load any planner that you want to use. Before we
  // can load the planner, we need two objects, a RobotModel
  // and a PlanningScene.
  // We will start by instantiating a
  // `RobotModelLoader`_
  // object, which will look up
  // the robot description on the ROS parameter server and construct a
  // :moveit_core:`RobotModel` for us to use.
  //
  // .. _RobotModelLoader: http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_ERROR);

  std::string urdf_file("pr2.urdf");
  std::string srdf_file("pr2.srdf");
  auto umodel = new urdf::Model();
  auto urdf_model = std::make_shared<urdf::ModelInterface>();
  urdf_model.reset(umodel);
  umodel->initFile(urdf_file);

  auto srdf_model = std::make_shared<srdf::Model>();
  srdf_model->initFile(*urdf_model, srdf_file);


  auto robot_model = std::make_shared<robot_model::RobotModel>(urdf_model, srdf_model);
  CONSOLE_BRIDGE_logInform("Model frame: %s", robot_model->getModelFrame().c_str());

  //ik plugin
  kinematics_plugin_loader::KinematicsPluginLoaderPtr kinematics_loader(
          new kinematics_plugin_loader::KinematicsPluginLoader());

  robot_model::SolverAllocatorFn kinematics_allocator = kinematics_loader->getLoaderFunction(srdf_model);

  const std::vector<std::string>& groups = kinematics_loader->getKnownGroups();

  std::stringstream ss;
  std::copy(groups.begin(), groups.end(), std::ostream_iterator<std::string>(ss, " "));
  CONSOLE_BRIDGE_logDebug("Loaded information about the following groups: %s" ,ss.str().c_str() );

  std::map<std::string, robot_model::SolverAllocatorFn> imap;
  CONSOLE_BRIDGE_logDebug("4");
  for (std::size_t i = 0; i < groups.size(); ++i)
  {
    // Check if a group in kinematics.yaml exists in the srdf
    if (!robot_model->hasJointModelGroup(groups[i]))
      continue;

    const robot_model::JointModelGroup* jmg = robot_model->getJointModelGroup(groups[i]);
    CONSOLE_BRIDGE_logDebug("5");
    kinematics::KinematicsBasePtr solver = kinematics_allocator(jmg);
    CONSOLE_BRIDGE_logDebug("6");
    if (solver)
    {
      std::string error_msg;
      if (solver->supportsGroup(jmg, &error_msg))
      {
        imap[groups[i]] = kinematics_allocator;
      }
      else
      {
        CONSOLE_BRIDGE_logError("Kinematics solver %s does not support joint group %s.  Error: %s", typeid(*solver).name(),
                                groups[i].c_str(), error_msg.c_str());
      }
    }
    else
    {
      CONSOLE_BRIDGE_logError("Kinematics solver could not be instantiated for joint group %s.", groups[i].c_str());
    }
  }
  robot_model->setKinematicsAllocators(imap);
  // end

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :planning_scene:`PlanningScene` that maintains the state of
  // the world (including the robot).
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
          new planning_pipeline::PlanningPipeline(robot_model, "planning_plugin",
                                                  "default_planner_request_adapters::AddTimeParameterization "
                  "default_planner_request_adapters::FixWorkspaceBounds "
                  "default_planner_request_adapters::FixStartStateBounds "
                                                          "default_planner_request_adapters::FixStartStateCollision"
                  "  default_planner_request_adapters::FixStartStatePathConstraints"));


  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(15.0);
  sleep_time.sleep();

  // Pose Goal
  // ^^^^^^^^^
  // We will now create a motion plan request for the right arm of the PR2
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "torso_lift_link";
  pose.pose.position.x = 0.75;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // We will create the request as a constraint using a helper function available
  // from the
  // `kinematic_constraints`_
  // package.
  //
  // .. _kinematic_constraints: http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
  req.group_name = "right_arm";
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  // We now construct a planning context that encapsulate the scene,
  // the request and the response. We call the planner using this
  // planning context

  planning_pipeline->generatePlan(planning_scene, req, res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    CONSOLE_BRIDGE_logError("Could not compute plan successfully");
    return 0;
  }
  // display
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);

  std::ostream s(std::cout.rdbuf());
  ros::message_operations::Printer< moveit_msgs::DisplayTrajectory >::stream(s, "", display_trajectory);
  // Joint Space Goals
  // ^^^^^^^^^^^^^^^^^
  /* First, set the state in the planning scene to the final state of the last plan */
  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  planning_scene->setCurrentState(response.trajectory_start);
  const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("right_arm");
  robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

  // Now, setup a joint space goal
  robot_state::RobotState goal_state(robot_model);
  std::vector<double> joint_values(7, 0.0);
  joint_values[0] = -2.0;
  joint_values[3] = -0.2;
  joint_values[5] = -0.15;
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // Call the planner and visualize the trajectory
  /* Re-construct the planning context */
  planning_pipeline->generatePlan(planning_scene, req, res);
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    CONSOLE_BRIDGE_logError("Could not compute plan successfully");
    return 0;
  }

  /* We will add more goals. But first, set the state in the planning
     scene to the final state of the last plan */
  robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

  /* Now, we go back to the first goal*/
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  planning_pipeline->generatePlan(planning_scene, req, res);

  res.getMessage(response);
  display_trajectory.trajectory.push_back(response.trajectory);

  // Adding Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // Let's add a new pose goal again. This time we will also add a path constraint to the motion.
  /* Let's create a new pose goal */
  pose.pose.position.x = 0.65;
  pose.pose.position.y = -0.2;
  pose.pose.position.z = -0.1;
  moveit_msgs::Constraints pose_goal_2 =
      kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", pose, tolerance_pose, tolerance_angle);
  /* First, set the state in the planning scene to the final state of the last plan */
  robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  /* Now, let's try to move to this new pose goal*/
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal_2);

  /* But, let's impose a path constraint on the motion.
     Here, we are asking for the end-effector to stay level*/
  geometry_msgs::QuaternionStamped quaternion;
  quaternion.header.frame_id = "base_link";
  quaternion.quaternion.w = 1.0;
  req.path_constraints = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", quaternion);

  // Imposing path constraints requires the planner to reason in the space of possible positions of the end-effector
  // (the workspace of the robot)
  // because of this, we need to specify a bound for the allowed planning volume as well;
  // Note: a default bound is automatically filled by the WorkspaceBounds request adapter (part of the OMPL pipeline,
  // but that is not being used in this example).
  // We use a bound that definitely includes the reachable space for the arm. This is fine because sampling is not done
  // in this volume
  // when planning for the arm; the bounds are only used to determine if the sampled configurations are valid.
  req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
      req.workspace_parameters.min_corner.z = -2.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
      req.workspace_parameters.max_corner.z = 2.0;

  // Call the planner and visualize all the plans created so far.
  planning_pipeline->generatePlan(planning_scene, req, res);
  res.getMessage(response);
  display_trajectory.trajectory.push_back(response.trajectory);
  // Now you should see four planned trajectories in series


  // END_TUTORIAL
  sleep_time.sleep();
  CONSOLE_BRIDGE_logInform("Done");

#endif
  return 0;
}
