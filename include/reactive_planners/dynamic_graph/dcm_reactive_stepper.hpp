/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implements the "Walking Control Based on Step Timing Adaptation" step
 * planner. The pdf can be found in https://arxiv.org/abs/1704.01271, and in the
 * `doc/` folder in this repository.
 */

#pragma once

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>

#include "reactive_planners/dcm_reactive_stepper.hpp"
#include "real_time_tools/mutex.hpp"

namespace reactive_planners
{
namespace dynamic_graph
{
/**
 * @brief Dynamic-graph entity wrapping the DcmReactiveStepper form the
 * reactive_stepping package.
 */
class DcmReactiveStepper : public dynamicgraph::Entity
{
    DYNAMIC_GRAPH_ENTITY_DECL();

public:
    /**
     * @brief Construct a new DcmReactiveStepper object using its Dynamic Graph
     * name.
     *
     * @param name
     */
    DcmReactiveStepper(const std::string &name);

    /** @brief @copydoc reactive_planners::DcmReactiveStepper::initialize() */
    void initialize(const bool &is_left_leg_in_contact,
                    const double &l_min,
                    const double &l_max,
                    const double &w_min,
                    const double &w_max,
                    const double &t_min,
                    const double &t_max,
                    const double &l_p,
                    const double &com_height,
                    Eigen::Ref<const Eigen::Vector9d> cost_weights_local,
                    const double &mid_air_foot_height,
                    const double &control_period,
                    const double &planner_loop,
                    Eigen::Ref<const Eigen::Vector3d> left_foot_position,
                    Eigen::Ref<const Eigen::Vector3d> right_foot_position);

    /**
     * @brief Command to initialize entity parameters.
     */
    void initializeParamVector(const dynamicgraph::Vector &parameters);

    /**
     * @brief Start the stepping.
     */
    void start()
    {
        start_stop_mutex_.lock();
        dcm_reactive_stepper_.start();
        start_stop_mutex_.unlock();
    }

    /**
     * @brief Stop the stepping.
     */
    void stop()
    {
        start_stop_mutex_.lock();
        dcm_reactive_stepper_.stop();
        start_stop_mutex_.unlock();
    }

public:
    /*
     * Input Signals
     */

    /** @brief Desired robot center of mass velocity. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        desired_com_velocity_sin_;

    /** @brief Current left foot position. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        current_left_foot_position_sin_;

    /** @brief Current right foot position. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        current_right_foot_position_sin_;

    /** @brief Current left foot velocity. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        current_left_foot_velocity_sin_;

    /** @brief Current right foot velocity. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        current_right_foot_velocity_sin_;

    /** @brief Current center of mass position. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> com_position_sin_;

    /** @brief Current center of mass velocity. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> com_velocity_sin_;

    /** @brief Current base yaw orientation. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> base_yaw_sin_;

    /** @brief Current base yaw orientation. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> xyzquat_base_sin_;

    /** @brief Current left foot position. Lhum closed loop. */
    dynamicgraph::SignalPtr<double, int> is_closed_loop_sin_;

    /*
     * Output Signals
     */

    /** @brief Desired right foot position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        right_foot_position_sout_;

    /** @brief Desired right foot velocity. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        right_foot_velocity_sout_;

    /** @brief Desired right foot acceleration. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        right_foot_acceleration_sout_;

    /** @brief Desired left foot position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        left_foot_position_sout_;

    /** @brief Desired left foot velocity. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        left_foot_velocity_sout_;

    /** @brief Desired left foot acceleration. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        left_foot_acceleration_sout_;

    /** @brief Desired local right foot position in local frame. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        local_right_foot_position_sout_;

    /** @brief Desired local right foot velocity in local frame. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        local_right_foot_velocity_sout_;

    /** @brief Desired local left foot position in local frame. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        local_left_foot_position_sout_;

    /** @brief Desired local left foot velocity in local frame. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        local_left_foot_velocity_sout_;

    /** @brief Desired base frame position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        local_frame_position_sout_;

    /** @brief Desired base frame position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        local_frame_velocity_sout_;

    /** @brief Feasible velocity computed from the desired stride length. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        feasible_com_velocity_sout_;

    /** @brief Previous support foot position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        previous_support_foot_position_sout_;

    /** @brief Current support foot position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        current_support_foot_position_sout_;

    /** @brief Next support foot position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        next_support_foot_position_sout_;

    /** @brief Current step duration. */
    dynamicgraph::SignalTimeDependent<double, int> step_duration_sout_;

    /** @brief Time elapsed from the last step touchdown. */
    dynamicgraph::SignalTimeDependent<double, int>
        time_from_last_step_touchdown_sout_;

    /** @brief Desired flying foot position. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        flying_foot_position_sout_;

    /** @brief Defines if the left or right foot is in contact. */
    dynamicgraph::SignalTimeDependent<int, int> is_left_leg_in_contact_sout_;

    /** @brief Indicate if the solver has found a good solution. */
    dynamicgraph::SignalTimeDependent<int, int> has_solution_sout_;

    /** @brief Dcm. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> dcm_sout_;

    /** @brief Force. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> force_sout_;

protected:
    /** @brief Inner signal to manage all other signals. */
    dynamicgraph::SignalTimeDependent<bool, int> inner_sout_;

    /*
     * Private Methods.
     */
    double double_support_time_ = 0.01;
    double time_from_double_support_started_ = 0.0;
    double last_yaw_ = 0.;
    int nb_switch_yaw_ = 0;

protected:
    /**
     * @brief Callback of the inner_sout_ signal.
     *
     * @param s
     * @param time
     * @return true
     * @return false
     */
    bool &inner(bool &s, int time);
    /**
     * @brief Callback of the dcm_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &dcm(dynamicgraph::Vector &s, int time);
    /**
     * @brief Callback of the force_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &force(dynamicgraph::Vector &s, int time);

    /**
     * @brief Callback of the right_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &right_foot_position(dynamicgraph::Vector &s,
                                              int time);

    /**
     * @brief Callback of the right_foot_velocity_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &right_foot_velocity(dynamicgraph::Vector &s,
                                              int time);

    /**
     * @brief Callback of the right_foot_acceleration_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &right_foot_acceleration(dynamicgraph::Vector &s,
                                                  int time);

    /**
     * @brief Callback of the left_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &left_foot_position(dynamicgraph::Vector &s, int time);

    /**
     * @brief Callback of the left_foot_velocity_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &left_foot_velocity(dynamicgraph::Vector &s, int time);

    /**
     * @brief Callback of the left_foot_acceleration_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &left_foot_acceleration(dynamicgraph::Vector &s,
                                                 int time);

    /**
     * @brief Callback of the local_left_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &local_left_foot_position(dynamicgraph::Vector &s,
                                                   int time);

    /**
     * @brief Callback of the local_left_foot_velocity_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &local_left_foot_velocity(dynamicgraph::Vector &s,
                                                   int time);

    /**
     * @brief Callback of the local_right_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &local_right_foot_position(dynamicgraph::Vector &s,
                                                    int time);

    /**
     * @brief Callback of the local_right_foot_velocity_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &local_right_foot_velocity(dynamicgraph::Vector &s,
                                                    int time);

    /**
     * @brief Callback of the feasible_com_velocity_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &feasible_com_velocity(dynamicgraph::Vector &s,
                                                int time);

    /**
     * @brief Callback of the previous_support_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &previous_support_foot_position(
        dynamicgraph::Vector &s, int time);

    /**
     * @brief Callback of the current_support_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &current_support_foot_position(dynamicgraph::Vector &s,
                                                        int time);

    /**
     * @brief Callback of the next_support_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &next_support_foot_position(dynamicgraph::Vector &s,
                                                     int time);

    /**
     * @brief Callback of the step_duration_sout_ signal.
     *
     * @param s
     * @param time
     * @return double&
     */
    double &step_duration(double &s, int time);

    /**
     * @brief Callback of the time_from_last_step_touchdown_sout_ signal.
     *
     * @param s
     * @param time
     * @return double&
     */
    double &time_from_last_step_touchdown(double &s, int time);

    /**
     * @brief Callback of the flying_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector &flying_foot_position(dynamicgraph::Vector &s,
                                               int time);

    /**
     * @brief Callback of the flying_foot_position_sout_ signal.
     *
     * @param s
     * @param time
     * @return int&
     */
    int &is_left_leg_in_contact(int &s, int time);

    /**
     * @brief Callback of the has_solution_sout_ signal.
     *
     * @param tau
     * @param t
     * @return dynamicgraph::Vector&
     */
    int &has_solution(int &s, int time);

    /**
     * @brief Helper to define the name of the signals.
     *
     * @param is_input_signal
     * @param signal_type
     * @param signal_name
     * @return std::string
     */
    std::string make_signal_string(const bool &is_input_signal,
                                   const std::string &signal_type,
                                   const std::string &signal_name);

    /*
     * Attribute
     */

    /** @brief Reactive step planner using DCM and VRP. */
    reactive_planners::DcmReactiveStepper dcm_reactive_stepper_;

    real_time_tools::RealTimeMutex start_stop_mutex_;

    /** @brief Robot control period. */
    double control_period_;

    /** @brief Check if the solvers have a solution. */
    int has_solution_;

    /*
     * Python commands.
     */
protected:
    /**
     * @brief
     */
    class InitializeCommand : public dynamicgraph::command::Command
    {
    public:
        /**
         * @brief Construct a new InitializeCommand object
         *
         * @param entity
         * @param docstring
         */
        InitializeCommand(DcmReactiveStepper &entity,
                          const std::string &docstring);
        /**
         * @brief
         *
         * @return Value
         */
        virtual dynamicgraph::command::Value doExecute();
    };
};

}  // namespace dynamic_graph
}  // namespace reactive_planners
