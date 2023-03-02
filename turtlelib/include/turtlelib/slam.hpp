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
    double max_n = 10; // add to constructor
    double max_size = 2*max_n+3; // add to constructor

    arma::colvec q_t{3, arma::fill::zeros};
    arma::colvec u_t_prev{3, arma::fill::zeros}; // maybe can delete thgis
    arma::colvec m_t{2*max_n, arma::fill::zeros};
    arma::colvec belief_t = arma::join_cols(q_t, m_t);
    arma::colvec belief_t_prev = arma::join_cols(q_t, m_t);
    arma::colvec belief_t_predict = arma::join_cols(q_t, m_t);
    double Q; // add to constructior
    arma::mat sigma_tq{3,3, arma::fill::zeros};
    arma::mat sigma_tm = 99999*arma::eye<arma::mat>(2*max_n,2*max_n);
    arma::mat sigma_t = arma::join_cols((arma::join_rows(sigma_tq,arma::zeros<arma::mat>(3,2*max_n))),(arma::join_rows(arma::zeros<arma::mat>(2*max_n,3),sigma_tm)));
    arma::mat sigma_t_prev = sigma_t;
    arma::mat sigma_t_predict = arma::join_cols((arma::join_rows(sigma_tq,arma::zeros<arma::mat>(3,2*max_n))),(arma::join_rows(arma::zeros<arma::mat>(2*max_n,3),sigma_tm)));
    double R; // add to constructor

public:
    /// \brief Initialize state to all 0's
    EKFSlam();

    /// @brief Initialize robot state to a known value
    /// @param q0 Initial robot position
    /// @param Q_in Input process covariance
    /// @param R_in Input sensor covariance
    EKFSlam(arma::vec q0, double Q_in, double R_in);

    /// @brief Initialize robot state
    /// @param Q_in Input process covariance
    /// @param R_in Input sensor covariance
    EKFSlam(double Q_in, double R_in);

    /// @brief 
    void predict(Twist2D u_t);

    void update(arma::vec m);

    arma::colvec get_qt();

    arma::colvec get_belief();

    arma::colvec get_belief_predict();

    arma::colvec get_h();

    arma::colvec get_mt_track();

    arma::mat H_output;
        arma::mat K_out;
        arma::mat cov_out;
    arma::mat z_out;
    arma::mat z_diff;
    arma::mat z_es;
    arma::mat obstacle_tracking{2, max_n, arma::fill::zeros}; // Matrix to keep track of if an object has been seen (j,n) j = obstacle idx, n = 1 or 0 corresponding to seen or not
arma::mat M_output;
arma::mat A_out;
arma::mat sigma_t_pred_get;

double rj_out, phi_j_out;
};

}

#endif
