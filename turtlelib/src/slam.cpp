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
            auto const tmp_delta_mat = arma::colvec{0.0, delta_x*std::cos(belief_t(0)), delta_x*std::sin(belief_t(0))};
            auto A = arma::zeros<arma::mat>(43,43);
            A(1,0) = delta_x*std::sin(belief_t(0));
            A(1,1) = delta_x*std::cos(belief_t(0));
            auto const A = arma::eye<arma::mat>(43,43) + A;
        }
        else{
            // if rotation and translation
            auto const tmp_delta_mat = arma::colvec{delta_theta, -(delta_x/delta_theta)*std::sin(belief_t(0))+(delta_x/delta_theta)*std::sin(belief_t(0)+delta_theta), (delta_x/delta_theta)*std::cos(belief_t(0))-(delta_x/delta_theta)*std::cos(belief_t(0)+delta_theta)};
            auto A = arma::zeros<arma::mat>(43,43);
            A(1,0) = -(delta_x/delta_theta)*std::cos(belief_t(0))+(delta_x/delta_theta)*std::cos(belief_t(0)+delta_theta);
            A(1,1) = -(delta_x/delta_theta)*std::sin(belief_t(0))+(delta_x/delta_theta)*std::sin(belief_t(0)+delta_theta);
            auto const A = arma::eye<arma::mat>(43,43) + A;
        }

        auto const big_u_matrix = arma::join_cols(tmp_delta_mat, arma::zeros<arma::colvec>(2*20));

        // Calculate prdicted state
        belief_t_predict = belief_t + big_u_matrix;

        // Calcualate Q matrix
        auto Q_bar = arma::zeros<arma::mat>(43,43);
        Q_bar(0,0) = Q;
        Q_bar(1,1) = Q;
        Q_bar(2,2) = Q;

        // Calculate predicted covariance
        sigma_t_predict = A*sigma_t_prev*A.t() + Q_bar;
    }

    void EKFSlam::update(arma::vec m){
        // m(0) = x, m(1) = y, m(2) = j

        // Measured sensor data
        auto const rj = std::sqrt((m(0))*(m(0))+(m(1))*(m(1)));
        auto const phi_j = std::atan2(m(1), m(0)) - belief_t_predict(0);
        auto const z = arma::colvec{rj, phi_j};

        // Estimated sensor data
        auto const mx_estimate = belief_t(1)+rj*std::cos(phi_j+belief_t(0));
        auto const my_estimate = belief_t(2)+rj*std::sin(phi_j+belief_t(0));
        auto const rj_estimate = std::sqrt((mx_estimate-belief_t_predict(1))*(mx_estimate-belief_t_predict(1))+(my_estimate-belief_t_predict(2))*(my_estimate-belief_t_predict(2)));
        auto const phi_j_estimate = std::atan2(my_estimate - belief_t_predict(2), mx_estimate - belief_t_predict(1)) - belief_t_predict(0);
        auto const z_estimate = arma::colvec{rj_estimate, phi_j_estimate};

        // Update the obstacles in the state estimate
        belief_t(m(2)+3) = m(0);
        belief_t(m(2)+4) = m(1);

        // Construct H matrix
        auto const delta_x = m(0) - belief_t_predict(1);
        auto const delta_y = m(1) - belief_t_predict(2);
        auto const d = delta_x*delta_x + delta_y*delta_y;
        auto const H_tmp_0 = arma::join_rows(arma::vec{0, (-delta_x/std::sqrt(d)), (-delta_y/std::sqrt(d))}, arma::vec{-1, (delta_y/d), (-delta_x/d)});
        auto const H_tmp_1 = arma::zeros<arm::mat>(2,2*(m(2)-1));
        auto const H_tmp_2 = arma::join_rows(arma::vec{(delta_x/std::sqrt(d)), (delta_y/std::sqrt(d))}, arma::vec{(delta_y/d), (delta_x/d)});
        auto const H_tmp_3 = arma::zeros<arm::mat>(2,2*(2*20-m(2)));
        H = arma::join_rows(H_tmp_0, H_tmp_1, H_tmp_2, H_tmp_3);

        // Compute Kalman Gain
        auto const R_mat = R*arma::eye<arma::mat>(H.size());
        auto Ki = sigma_t_predict*H.t()*(H*sigma_t_predict*H.t()+R_mat).i();

        // Compute posterior state
        belief_t = belief_t_predict + Ki*(z - z_estimate);

        // Compute posterior covariance
        sigma_t = (arma::eye<arma::mat>(H.size()); - Ki*H)*sigma_t_predict;
    }

}
