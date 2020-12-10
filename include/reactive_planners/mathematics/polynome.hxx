/**
 * @file polynome_impl.hh
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief Polynomes object for trajectories.
 * @version 0.1
 * @date 2019-11-06
 *
 * @copyright Copyright (c) 2019
 *
 */

#pragma once

#include <iostream>
#include <vector>
#include "blmc_robots/mathematics/polynome.hpp"

namespace blmc_robots
{
/**
 * Polynome<ORDER> definitions
 */

template <int ORDER>
Polynome<ORDER>::Polynome()
{
    coefficients_.fill(0.0);
}

template <int ORDER>
Polynome<ORDER>::~Polynome()
{
}

template <int ORDER>
double Polynome<ORDER>::compute(double x)
{
    double res = 0.0;
    double pt = 1.0;
    for (size_t i = 0; i < coefficients_.size(); ++i)
    {
        res += coefficients_[i] * pt;
        pt *= x;
    }
    return res;
}

template <int ORDER>
double Polynome<ORDER>::compute_derivative(double x)
{
    double res = 0.0;
    double pt = 1.0;
    for (size_t i = 1; i < coefficients_.size(); ++i)
    {
        res += i * coefficients_[i] * pt;
        pt *= x;
    }
    return res;
}

template <int ORDER>
double Polynome<ORDER>::compute_sec_derivative(double x)
{
    double res = 0.0;
    double pt = 1.0;
    for (size_t i = 2; i < coefficients_.size(); ++i)
    {
        res += i * (i - 1) * coefficients_[i] * pt;
        pt *= x;
    }
    return res;
}

template <int ORDER>
void Polynome<ORDER>::get_coefficients(Coefficients &coefficients) const
{
    coefficients = coefficients_;
}

template <int ORDER>
void Polynome<ORDER>::set_coefficients(const Coefficients &coefficients)
{
    coefficients_ = coefficients;
}

template <int ORDER>
void Polynome<ORDER>::print() const
{
    for (size_t i = 0; i < ORDER; ++i)
    {
        std::cout << coefficients_[i] << " ";
    }
    std::cout << std::endl;
}

/**
 * TimePolynome<ORDER> definitions
 */

template <int ORDER>
double TimePolynome<ORDER>::compute(double t)
{
    if (t <= 0.0)
    {
        return init_pose_;
    }
    else if (t >= final_time_)
    {
        return final_pose_;
    }
    else
    {
        return Polynome<ORDER>::compute(t);
    }
}

template <int ORDER>
double TimePolynome<ORDER>::compute_derivative(double t)
{
    if (t <= 0.0)
    {
        return init_speed_;
    }
    else if (t >= final_time_)
    {
        return final_speed_;
    }
    else
    {
        return Polynome<ORDER>::compute_derivative(t);
    }
}

template <int ORDER>
double TimePolynome<ORDER>::compute_sec_derivative(double t)
{
    if (t <= 0.0)
    {
        return init_acc_;
    }
    else if (t >= final_time_)
    {
        return final_acc_;
    }
    else
    {
        return Polynome<ORDER>::compute_sec_derivative(t);
    }
}

}  // namespace blmc_robots
