/**
 * @file polynome.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief Polynomes object for trajectories.
 *
 * @version 0.1
 * @date 2019-11-07
 *
 * @copyright Copyright (c) 2019
 *
 * See
 * https://github.com/jrl-umi3218/jrl-walkgen/blob/master/src/Mathematics/PolynomeFoot.cpp
 * for further enhancement.
 */

#include <blmc_robots/mathematics/polynome.hpp>
#include <iostream>

namespace blmc_robots
{
template <>
void TimePolynome<5>::SetParameters(double final_time,
                                    double init_pos,
                                    double init_speed,
                                    double init_acc,
                                    double final_pos,
                                    double final_speed,
                                    double final_acc)
{
    double tmp;
    m_Coefficients[0] = InitPos_ = InitPos;
    m_Coefficients[1] = InitSpeed_ = InitSpeed;
    m_Coefficients[2] = InitAcc / 2.0;
    InitAcc_ = InitAcc;
    FT_ = FT;
    FinalPos_ = FinalPos;
    tmp = FT * FT * FT;
    if (tmp == 0.0)
    {
        m_Coefficients[3] = 0.0;
        m_Coefficients[4] = 0.0;
        m_Coefficients[5] = 0.0;
    }
    else
    {
        m_Coefficients[3] =
            -(1.5 * InitAcc * FT * FT - 0.5 * FinalAcc * FT * FT +
              6.0 * InitSpeed * FT + 4.0 * FinalSpeed * FT + 10.0 * InitPos -
              10.0 * FinalPos) /
            tmp;
        tmp = tmp * FT;
        m_Coefficients[4] = (1.5 * InitAcc * FT * FT - FinalAcc * FT * FT +
                             8.0 * InitSpeed * FT + 7.0 * FinalSpeed * FT +
                             15.0 * InitPos - 15.0 * FinalPos) /
                            tmp;
        tmp = tmp * FT;
        m_Coefficients[5] =
            -(0.5 * InitAcc * FT * FT - 0.5 * FinalAcc * FT * FT +
              3.0 * InitSpeed * FT + 3.0 * FinalSpeed * FT + 6.0 * InitPos -
              6.0 * FinalPos) /
            tmp;
    }
}

}  // namespace blmc_robots