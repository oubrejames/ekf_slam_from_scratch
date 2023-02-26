#include "turtlelib/slam.hpp"
#include <iostream>
#include <climits>

namespace turtlelib
{
    EKFSlam::EKFSlam() :
        q_t{3, arma::fill::zeros}
    {}

    EKFSlam::EKFSlam(arma::vec q0_in) :
        q_t{q0_in}
    {}

    void EKFSlam::predict(Twist2D u_t){
        // // Integrate the odometry twist to get the actual wheel positions
        // auto const integrated_odom = turtlelib::integrate_twist(u_t);

        // // Subtract prev odom from current to get the delta u
        // auto const delta_x = integrated_odom.translation().x - u_t_prev(1);
        // auto const delta_theta = integrated_odom.rotation() - u_t_prev(0);

        // // Update previous u
        // u_t_prev = {integrated_odom.rotation(), integrated_odom.translation().x, integrated_odom.translation().y};


        // Subtract prev odom from current to get the delta u
        auto const delta_x = u_t.x - u_t_prev(1);
        auto const delta_theta = turtlelib::normalize_angle(u_t.w - u_t_prev(0));

        // Update previous u
        u_t_prev = {u_t.w, u_t.x, u_t.y};

        if(turtlelib::almost_equal(delta_theta, 0.0)){
            // No rotation
            auto const tmp_delta_mat = arma::colvec{0.0, delta_x*std::cos(belief_t_prev(0)), delta_x*std::sin(belief_t_prev(0))};
            auto A = arma::zeros<arma::mat>(43,43);
            A(1,0) = delta_x*std::sin(belief_t_prev(0));
            A(1,1) = delta_x*std::cos(belief_t_prev(0));
            auto const A = arma::eye<arma::mat>(43,43) + A;
        }
        else{
            // if rotation and translation
            auto const tmp_delta_mat = arma::colvec{delta_theta, -(delta_x/delta_theta)*std::sin(belief_t_prev(0))+(delta_x/delta_theta)*std::sin(belief_t_prev(0)+delta_theta), (delta_x/delta_theta)*std::cos(belief_t_prev(0))-(delta_x/delta_theta)*std::cos(belief_t_prev(0)+delta_theta)};
            auto A = arma::zeros<arma::mat>(43,43);
            A(1,0) = -(delta_x/delta_theta)*std::cos(belief_t_prev(0))+(delta_x/delta_theta)*std::cos(belief_t_prev(0)+delta_theta);
            A(1,1) = -(delta_x/delta_theta)*std::sin(belief_t_prev(0))+(delta_x/delta_theta)*std::sin(belief_t_prev(0)+delta_theta);
            auto const A = arma::eye<arma::mat>(43,43) + A;
        }

        auto const big_u_matrix = arma::join_cols(tmp_delta_mat, arma::zeros<arma::colvec>(2*20));

        // Calculate prdicted state
        belief_t_predict = belief_t_prev + big_u_matrix;

        // Calcualate Q matrix
        auto Q_bar = arma::zeros<arma::mat>(43,43);
        Q_bar(0,0) = Q;
        Q_bar(1,1) = Q;
        Q_bar(2,2) = Q;

        // Calculate predicted covariance
        sigma_t_predict = A*sigma_t_prev*A.t() + Q_bar;
    }
}
