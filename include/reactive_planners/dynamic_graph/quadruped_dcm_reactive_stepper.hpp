/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implements the "Walking Control Based on Step Timing Adaptation" step
 * planner for a quadruped.
 */

#pragma once

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>

#include "reactive_planners/quadruped_dcm_reactive_stepper.hpp"
#include "real_time_tools/mutex.hpp"

namespace reactive_planners
{
namespace dynamic_graph
{
/**
 * @brief Dynamic-graph entity wrapping the QuadrupedDcmReactiveStepper form the
 * reactive_stepping package.
 */
class QuadrupedDcmReactiveStepper : public dynamicgraph::Entity
{
    DYNAMIC_GRAPH_ENTITY_DECL();

public:
    /**
     * @brief Construct a new QuadrupedDcmReactiveStepper object using its
     * Dynamic Graph name.
     *
     * @param name Entity name.
     */
    QuadrupedDcmReactiveStepper(const std::string& name);

    /**
     * @brief Set the nominal steptime.
     */
    void set_steptime_nominal(double t_nom);

    /** @brief Set to use polynominal interpolation for the endeffector
     * trajectories. */
    void set_polynomial_end_effector_trajectory();

    /** @brief Set to use the mpc based method for the endeffector trajectories
     */
    void set_dynamical_end_effector_trajectory();

    /** @brief Starts the reactive stepping */
    void start();

    /** @brief Stops the reactive stepping */
    void stop();

    /**
     * @brief Initializes the placement information of the robot.
     *
     * @param base_placement 7d Vector representing the base placement.
     * @param front_left_foot_position Position of the center of the foot.
     * @param front_right_foot_position Position of the center of the foot.
     * @param hind_left_foot_position Position of the center of the foot.
     * @param hind_right_foot_position Position of the center of the foot.
     */
    void initialize_placement(
        const Eigen::Ref<const Eigen::Vector7d>& base_placement,
        const Eigen::Ref<const Eigen::Vector3d>& front_left_foot_position,
        const Eigen::Ref<const Eigen::Vector3d>& front_right_foot_position,
        const Eigen::Ref<const Eigen::Vector3d>& hind_left_foot_position,
        const Eigen::Ref<const Eigen::Vector3d>& hind_right_foot_position);

    /**
     * @brief Initializes the parameters for the reactive stepper.
     *
     * @param is_left_leg_in_contact Boolean to indicate which leg is in
     * contact at the initialization phase.
     * @param l_min [in] Minimum step length in the x direction (in the
     * direction of forward motion).
     * @param l_max [in] Maximum step length in the x direction (in the
     * direction of forward motion).
     * @param w_min [in] Minimum step length in the y direction (in the lateral
     * direction).
     * @param w_max [in] Maximum step length in the y direction (in the lateral
     * direction).
     * @param t_min [in] Minimum step time.
     * @param t_max [in] Maximum step time.
     * @param l_p [in] Default lateral step length. Typically useful for
     * humanoid robot where this value refer to the distance between the 2 feet
     * while in the half-sitting/neutral position.
     * @param com_height [in] Average desired height of the com above the
     * ground.
     * @param weight [in] Weights of the QP cost.
     * @param mid_air_foot_height [in] Apex of the flying foot height.
     * @param control_period [in] Control period in seconds.
     * @param planner_loop [in] Period of the planner in seconds
     */
    void initialize_stepper(const bool& is_left_leg_in_contact,
                            const double& l_min,
                            const double& l_max,
                            const double& w_min,
                            const double& w_max,
                            const double& t_min,
                            const double& t_max,
                            const double& l_p,
                            const double& com_height,
                            const Eigen::Vector9d& weight,
                            const double& mid_air_foot_height,
                            const double& control_period,
                            const double& planner_loop);

    /** @brief Initializes the placement and reactive stepper at once.
     *
     * @copydoc QuadrupedDcmReactiveStepper::initialize_placement()
     * @copydoc QuadrupedDcmReactiveStepper::initialize_stepper()
     */
    void initialize(
        const bool& is_left_leg_in_contact,
        const double& l_min,
        const double& l_max,
        const double& w_min,
        const double& w_max,
        const double& t_min,
        const double& t_max,
        const double& l_p,
        const double& com_height,
        const Eigen::Vector9d& weight,
        const double& mid_air_foot_height,
        const double& control_period,
        const double& planner_loop,
        const Eigen::Ref<const Eigen::Vector7d>& base_placement,
        const Eigen::Ref<const Eigen::Vector3d>& front_left_foot_position,
        const Eigen::Ref<const Eigen::Vector3d>& front_right_foot_position,
        const Eigen::Ref<const Eigen::Vector3d>& hind_left_foot_position,
        const Eigen::Ref<const Eigen::Vector3d>& hind_right_foot_position);

public:
    /*
     * Input Signals
     */

    /** @brief Current quadruped front left foot position. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        current_front_left_foot_position_sin_;

    /** @brief Current quadruped front right foot position. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        current_front_right_foot_position_sin_;

    /** @brief Current quadruped hind left foot position. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        current_hind_left_foot_position_sin_;

    /** @brief Current quadruped hind right foot position. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        current_hind_right_foot_position_sin_;

    /** @brief Current quadruped front left foot velocity. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        current_front_left_foot_velocity_sin_;

    /** @brief Current quadruped front right foot velocity. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        current_front_right_foot_velocity_sin_;

    /** @brief Current quadruped hind left foot velocity. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        current_hind_left_foot_velocity_sin_;

    /** @brief Current quadruped hind right foot velocity. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        current_hind_right_foot_velocity_sin_;

    /** @brief Current quadruped com position. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> com_position_sin_;

    /** @brief Current quadruped com velocity in world frame. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> com_velocity_sin_;

    /** @brief Current quadruped base SE3 posture. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> xyzquat_base_sin_;

    /** @brief ?????? . */
    dynamicgraph::SignalPtr<double, int> is_closed_loop_sin_;

    /**
     * @brief Desired COM velocity to track with the stepper in local yaw frame
     * of the base.
     */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        desired_com_velocity_sin_;

    /*
     * Output Signals
     */
    /** @brief Desired front left foot position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        front_left_foot_position_sout_;

    /** @brief Desired front right foot position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        front_right_foot_position_sout_;

    /** @brief Desired hind left foot position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        hind_left_foot_position_sout_;

    /** @brief Desired hind right foot position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        hind_right_foot_position_sout_;

    /** @brief Desired front left foot velocity. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        front_left_foot_velocity_sout_;

    /** @brief Desired front right foot velocity. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        front_right_foot_velocity_sout_;

    /** @brief Desired hind left foot velocity. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        hind_left_foot_velocity_sout_;

    /** @brief Desired hind right foot velocity. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        hind_right_foot_velocity_sout_;

    /** @brief Desired front left foot acceleration. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        front_left_foot_acceleration_sout_;

    /** @brief Desired front right foot acceleration. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        front_right_foot_acceleration_sout_;

    /** @brief Desired hind left foot acceleration. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        hind_left_foot_acceleration_sout_;

    /** @brief Desired hind right foot acceleration. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        hind_right_foot_acceleration_sout_;

    /** @brief Feasible com velocity. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        feasible_com_velocity_sout_;

    /** @brief Active endeffector contacts. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        contact_array_sout_;

    /** @brief Feedforward force for the swing foot. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        swing_foot_forces_sout_;

protected:
    /** @brief True if the placement was initialized, false otherwise. */
    bool init_placement_;

    /** @brief The base placement provided during the initialization. */
    Eigen::Vector7d init_base_placement_;

    /** @brief The init front left foot position provided during
     * initialization. */
    Eigen::Vector3d init_front_left_foot_position_;

    /** @brief The init front right foot position provided during
     * initialization. */
    Eigen::Vector3d init_front_right_foot_position_;

    /** @brief The init hind left foot position provided during
     * initialization. */
    Eigen::Vector3d init_hind_left_foot_position_;

    /** @brief The init hind right foot position provided during
     * initialization. */
    Eigen::Vector3d init_hind_right_foot_position_;

    /** @brief Quaternion from the input signal. */
    pinocchio::SE3::Quaternion base_quaternion_;

protected:
    /**
     * @brief Callback of the inner_sout_ signal.
     *
     * @param s
     * @param time
     */
    bool& inner(bool& s, int time);

    /**
     * @brief Callback of the front_left_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& front_left_foot_position(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the front_right_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& front_right_foot_position(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the hind_left_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& hind_left_foot_position(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the hind_right_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& hind_right_foot_position(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the front_left_foot_velocity_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& front_left_foot_velocity(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the front_right_foot_velocity_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& front_right_foot_velocity(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the hind_left_foot_velocity_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& hind_left_foot_velocity(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the hind_right_foot_velocity_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& hind_right_foot_velocity(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the front_left_foot_acceleration_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& front_left_foot_acceleration(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the front_right_foot_acceleration_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& front_right_foot_acceleration(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the hind_left_foot_acceleration_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& hind_left_foot_acceleration(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the hind_right_foot_acceleration_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& hind_right_foot_acceleration(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the hind_feasible_com_velocity_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& feasible_com_velocity(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback of the contact_array_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& contact_array(dynamicgraph::Vector& signal_data,
                                        int time);

    /**
     * @brief Callback of the swing_foot_forces_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& swing_foot_forces(dynamicgraph::Vector& signal_data,
                                            int time);

    /**
     * @brief Helper to define the name of the signals.
     *
     * @param is_input_signal
     * @param signal_type
     * @param signal_name
     * @return std::string
     */
    std::string make_signal_string(const bool& is_input_signal,
                                   const std::string& signal_type,
                                   const std::string& signal_name);

protected:
    /** @brief Inner signal to manage all other signals. */
    dynamicgraph::SignalTimeDependent<bool, int> inner_sout_;

    /** @brief The central quadruped reactive stepper controller */
    reactive_planners::QuadrupedDcmReactiveStepper stepper_;
};

}  // namespace dynamic_graph
}  // namespace reactive_planners
