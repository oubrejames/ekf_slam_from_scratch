#ifndef SLAM_INCLUDE_GUARD_HPP
#define SLAM_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.

#include "turtlelib/rigid2d.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include <armadillo>

namespace turtlelib{
    
/// \brief a rigid body transformation in 2 dimensions
class EKFSlam
{
private:
    arma::colvec q_t{3, arma::fill::zeros};
    arma::colvec u_t_prev{3, arma::fill::zeros}; // maybe can delete thgis
    arma::colvec m_t{20, arma::fill::zeros};
    arma::colvec belief_t = arma::join_cols(q_t, m_t);
    arma::colvec belief_t_prev = arma::join_cols(q_t, m_t);
    arma::colvec belief_t_predict = arma::join_cols(q_t, m_t);
    arma::mat A{43, 43, arma::fill::zeros};
    arma::mat H;
    arma::mat h_j;
    double Q;
    arma::mat sigma_tq{3,3, arma::fill::zeros};
    arma::mat sigma_tm = 99999*arma::eye<arma::mat>(2*20,2*20);
    arma::mat sigma_t = arma::join_cols((arma::join_rows(sigma_tq,arma::zeros<arma::mat>(3,2*20))),(arma::join_rows(arma::zeros<arma::mat>(2*20,3),sigma_tm)));
    arma::mat sigma_t_prev = sigma_t;
    arma::mat sigma_t_predict = arma::join_cols((arma::join_rows(sigma_tq,arma::zeros<arma::mat>(3,2*20))),(arma::join_rows(arma::zeros<arma::mat>(2*20,3),sigma_tm)));

public:
    /// \brief Initialize state to all 0's
    EKFSlam();

    /// @brief Initialize robot state to a known value
    /// @param q0 Initial robot position
    explicit EKFSlam(arma::vec q0);

    /// @brief 
    void predict(Twist2D u_t);
};

}

#endif
