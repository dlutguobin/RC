//
// Created by siasun on 18-1-25.
//

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <console_bridge/console.h>
#include <iostream>
#include <memory>

int main(int argc, char **argv)
{

    // BEGIN_TUTORIAL
    // Start
    // ^^^^^
    // Setting up to start using the RobotModel class is very easy. In
    // general, you will find that most higher-level components will
    // return a shared pointer to the RobotModel. You should always use
    // that when possible. In this example, we will start with such a
    // shared pointer and discuss only the basic API. You can have a
    // look at the actual code API for these classes to get more
    // information about how to use more features provided by these
    // classes.
    //
    // We will start by instantiating a
    // `RobotModelLoader`_
    // object, which will look up
    // the robot description on the ROS parameter server and construct a
    // :moveit_core:`RobotModel` for us to use.
    //
    // .. _RobotModelLoader: http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
    //robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    //ModelInterfaceSharedPtr urdf_model(new ModelInterface());
    console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
    std::string urdf_file("srcz.urdf");
    std::string srdf_file("srcz.srdf");
    auto umodel = new urdf::Model();
    auto urdf_model = std::make_shared<urdf::ModelInterface>();
    urdf_model.reset(umodel);
    umodel->initFile(urdf_file);

    auto srdf_model = std::make_shared<srdf::Model>();
    srdf_model->initFile(*urdf_model, srdf_file);


    auto kinematic_model = std::make_shared<robot_model::RobotModel>(urdf_model, srdf_model);
    CONSOLE_BRIDGE_logInform("Model frame: %s", kinematic_model->getModelFrame().c_str());

    kinematics_plugin_loader::KinematicsPluginLoaderPtr kinematics_loader(
            new kinematics_plugin_loader::KinematicsPluginLoader());
    CONSOLE_BRIDGE_logDebug("1");
    robot_model::SolverAllocatorFn kinematics_allocator = kinematics_loader->getLoaderFunction(srdf_model);
    CONSOLE_BRIDGE_logDebug("2");
    const std::vector<std::string>& groups = kinematics_loader->getKnownGroups();
    CONSOLE_BRIDGE_logDebug("3");
    std::stringstream ss;
    std::copy(groups.begin(), groups.end(), std::ostream_iterator<std::string>(ss, " "));
    CONSOLE_BRIDGE_logDebug("Loaded information about the following groups: %s" ,ss.str().c_str() );

    std::map<std::string, robot_model::SolverAllocatorFn> imap;
    CONSOLE_BRIDGE_logDebug("4");
    for (std::size_t i = 0; i < groups.size(); ++i)
    {
        // Check if a group in kinematics.yaml exists in the srdf
        if (!kinematic_model->hasJointModelGroup(groups[i]))
            continue;

        const robot_model::JointModelGroup* jmg = kinematic_model->getJointModelGroup(groups[i]);
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
    kinematic_model->setKinematicsAllocators(imap);

    // set the default IK timeouts
    const std::map<std::string, double>& timeout = kinematics_loader->getIKTimeout();
    for (std::map<std::string, double>::const_iterator it = timeout.begin(); it != timeout.end(); ++it)
    {
        if (!kinematic_model->hasJointModelGroup(it->first))
            continue;
        robot_model::JointModelGroup* jmg = kinematic_model->getJointModelGroup(it->first);
        jmg->setDefaultIKTimeout(it->second);
    }

    // set the default IK attempts
    const std::map<std::string, unsigned int>& attempts = kinematics_loader->getIKAttempts();
    for (std::map<std::string, unsigned int>::const_iterator it = attempts.begin(); it != attempts.end(); ++it) {
        if (!kinematic_model->hasJointModelGroup(it->first))
            continue;
        robot_model::JointModelGroup *jmg = kinematic_model->getJointModelGroup(it->first);
        jmg->setDefaultIKAttempts(it->second);
    }


    // Using the :moveit_core:`RobotModel`, we can construct a
    // :moveit_core:`RobotState` that maintains the configuration
    // of the robot. We will set all joints in the state to their
    // default values. We can then get a
    // :moveit_core:`JointModelGroup`, which represents the robot
    // model for a particular group, e.g. the "right_arm" of the PR2
    // robot.
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("arm");

    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    // Get Joint Values
    // ^^^^^^^^^^^^^^^^
    // We can retreive the current set of joint values stored in the state for the right arm.
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        CONSOLE_BRIDGE_logInform("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // Joint Limits
    // ^^^^^^^^^^^^
    // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
    /* Set one joint in the right arm outside its joint limit */
    joint_values[1] = 3.14;
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

    /* Check whether any joint is outside its joint limits */
    if (kinematic_state->satisfiesBounds())
    {
        CONSOLE_BRIDGE_logInform("Current state is valid");
    }
    else
    {
        CONSOLE_BRIDGE_logInform("Current state is not valid");
    }


    /* Enforce the joint limits for this state and check again*/
    kinematic_state->enforceBounds();
    if (kinematic_state->satisfiesBounds())
    {
        CONSOLE_BRIDGE_logInform("Current state is valid");
    }
    else
    {
        CONSOLE_BRIDGE_logInform("Current state is not valid");
    }

    // Forward Kinematics
    // ^^^^^^^^^^^^^^^^^^
    // Now, we can compute forward kinematics for a set of random joint
    // values. Note that we would like to find the pose of the
    // "r_wrist_roll_link" which is the most distal link in the
    // "right_arm" of the robot.
    kinematic_state->setToRandomPositions(joint_model_group);
    //double *j;
    const double *j = kinematic_state->getJointPositions("J1");
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("L6");

    /* Print end-effector pose. Remember that this is in the model frame */
    std::cout << end_effector_state.translation() << std::endl;
    std::cout << end_effector_state.rotation() << std::endl;

    // Inverse Kinematics
    // ^^^^^^^^^^^^^^^^^^
    // We can now solve inverse kinematics (IK) for the right arm of the
    // PR2 robot. To solve IK, we will need the following:
    // * The desired pose of the end-effector (by default, this is the last link in the "right_arm" chain):
    // end_effector_state that we computed in the step above.
    // * The number of attempts to be made at solving IK: 5
    // * The timeout for each attempt: 0.1 s
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

    // Now, we can print out the IK solution (if found):
    if (found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
            CONSOLE_BRIDGE_logInform("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }
    else
    {
        CONSOLE_BRIDGE_logInform("Did not find IK solution");
    }

    // Get the Jacobian
    // ^^^^^^^^^^^^^^^^
    // We can also get the Jacobian from the :moveit_core:`RobotState`.
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group,
                                 kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                 reference_point_position, jacobian);
    std::cout << "Jacobian: " << jacobian << std::endl;
    // END_TUTORIAL

    return 0;
}
