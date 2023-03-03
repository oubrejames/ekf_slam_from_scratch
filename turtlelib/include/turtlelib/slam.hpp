#ifndef SLAM_INCLUDE_GUARD_HPP
#define SLAM_INCLUDE_GUARD_HPP
/// \file
/// \brief EKF SLAM methods

#include "turtlelib/rigid2d.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include <armadillo>

namespace turtlelib{
    
/// \brief Class to impliment EKF SLAM functionality 
class EKFSlam
{
private:
    double max_n = 10;
    double max_size = 2*max_n+3;

    arma::colvec q_t{3, arma::fill::zeros};
    arma::colvec u_t_prev{3, arma::fill::zeros};
    arma::colvec m_t{2*max_n, arma::fill::zeros};
    arma::colvec belief_t = arma::join_cols(q_t, m_t);
    arma::colvec belief_t_prev = arma::join_cols(q_t, m_t);
    arma::colvec belief_t_predict = arma::join_cols(q_t, m_t);
    double Q;
    arma::mat sigma_tq{3,3, arma::fill::zeros};
    arma::mat sigma_tm = 99999*arma::eye<arma::mat>(2*max_n,2*max_n);
    arma::mat sigma_t = arma::join_cols((arma::join_rows(sigma_tq,arma::zeros<arma::mat>(3,2*max_n))),(arma::join_rows(arma::zeros<arma::mat>(2*max_n,3),sigma_tm)));
    arma::mat sigma_t_prev = sigma_t;
    arma::mat sigma_t_predict = arma::join_cols((arma::join_rows(sigma_tq,arma::zeros<arma::mat>(3,2*max_n))),(arma::join_rows(arma::zeros<arma::mat>(2*max_n,3),sigma_tm)));
    double R;
    arma::mat obstacle_tracking{2, max_n, arma::fill::zeros}; // Matrix to keep track of if an object has been seen (j,n) j = obstacle idx, n = 1 or 0 corresponding to seen or not

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

    /// @brief Calculate the predicted state estimate
    /// @param u_t Current odometry estimate of the robot
    void predict(Twist2D u_t);

    /// @brief Calculate the corrected state estimate of the robot
    /// @param m A vector holding the x and y coords of an obstacle along with its id
    void update(arma::vec m);

    /// @brief getter for current estimated position
    /// @return current estimated position
    arma::colvec get_qt();

    /// @brief getter for current estimated state
    /// @return current estimated state
    arma::colvec get_belief();

    /// @brief getter for current estimated state prediction
    /// @return current estimated state prediction
    arma::colvec get_belief_predict();

    /// @brief getter for obstacle id flags
    /// @return obstacle id flags
    arma::colvec get_mt_track();

};

}

#endif
