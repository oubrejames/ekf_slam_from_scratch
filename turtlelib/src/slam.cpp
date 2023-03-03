#include "turtlelib/slam.hpp"
#include <iostream>
#include <climits>

namespace turtlelib
{
    EKFSlam::EKFSlam() :
        q_t{3, arma::fill::zeros},
        Q{0.001},
        R{0.001}
    {}


    EKFSlam::EKFSlam(arma::vec q0, double Q_in, double R_in) :
        q_t{q0},
        Q{Q_in},
        R{R_in}
    {}

    EKFSlam::EKFSlam(double Q_in, double R_in) :
        Q{Q_in},
        R{R_in}
    {}

    void EKFSlam::predict(Twist2D u_t){
        // // Integrate the odometry twist to get the actual wheel positions
        // auto const integrated_odom = turtlelib::integrate_twist(u_t);

        // // Subtract prev odom from current to get the delta u
        // auto const delta_x = integrated_odom.translation().x - u_t_prev(1);
        // auto const delta_theta = integrated_odom.rotation() - u_t_prev(0);

        // // Update previous u
        // u_t_prev = {integrated_odom.rotation(), integrated_odom.translation().x, integrated_odom.translation().y};

        // uT PREV SHOULD BE SLAM STATES CURRENT IS JUST ODOMETRY - KT
        // Subtract prev odom from current to get the delta u
        auto const delta_x = u_t.x - u_t_prev(1);
        auto const delta_theta = turtlelib::normalize_angle(u_t.w - u_t_prev(0));
        auto const delta_y = u_t.y - u_t_prev(2);

        // Update previous u
        u_t_prev = {u_t.w, u_t.x, u_t.y};

        // Construct A matrix
        arma::mat A{2*max_n+3, 2*max_n+3, arma::fill::eye};
        A(1,0) = -delta_y;
        A(2,0) = delta_x;
        A_out = A;

        belief_t(0) = belief_t(0) + delta_theta;
        belief_t(1) = belief_t(1) + delta_x;
        belief_t(2) = belief_t(2) + delta_y;

        // Calcualate Q matrix
        arma::mat Q_bar{max_size, max_size, arma::fill::zeros};
        Q_bar(0,0) = Q;
        Q_bar(1,1) = Q;
        Q_bar(2,2) = Q;

        // Calculate predicted covariance
        sigma_t = A*sigma_t*A.t() + Q_bar;
        sigma_t_pred_get = sigma_t;

    }

    void EKFSlam::update(arma::vec m){
        // m(0) = x, m(1) = y, m(2) = j

        // Measured sensor data
        auto const rj = std::sqrt((m(0))*(m(0))+(m(1))*(m(1)));
        auto const phi_j = (atan2(m(1), m(0)));
        arma::colvec z{rj, phi_j};
        rj_out = rj;

        // Initialize obstacles I havent seen before
        if((obstacle_tracking(1, m(2))) < 1){
            belief_t(2*m(2)+3) = belief_t(1)+rj*std::cos(phi_j+belief_t(0));
            belief_t(2*m(2)+4) = belief_t(2)+rj*std::sin(phi_j+belief_t(0));
            obstacle_tracking(1, m(2)) = 1;
        }
        
        
        // Calculate H matrix intermediate values
        auto const delta_x = belief_t(2*m(2)+3) - belief_t(1);
        auto const delta_y = belief_t(2*m(2)+4) - belief_t(2);
        auto const d = delta_x*delta_x + delta_y*delta_y;

        // Theoretical sensor data
        auto const rj_estimate = std::sqrt(d);
        auto const phi_j_estimate =turtlelib::normalize_angle(atan2(delta_y, delta_x)-belief_t(0));

        arma::colvec z_estimate{rj_estimate, phi_j_estimate};

        // Calculate H matrix
        arma::mat H_tmp_0 = {{0.0, (-delta_x/std::sqrt(d)), (-delta_y/std::sqrt(d))}, 
                             {-1.0, (delta_y/d), (-delta_x/d)}};

        arma::mat H_tmp_1{2, static_cast<arma::uword>(2*(m(2))), arma::fill::zeros};

        arma::mat H_tmp_2 = {{(delta_x/std::sqrt(d)), (delta_y/std::sqrt(d))}, 
                            {-(delta_y/d), (delta_x/d)}};

        arma::mat H_tmp_3{2, static_cast<arma::uword>(2*(max_n-(m(2)+1))), arma::fill::zeros};

        arma::mat H = arma:: join_rows(H_tmp_0, H_tmp_1, H_tmp_2, H_tmp_3);

        H_output = H;

        // Compute Kalman Gain
        arma::mat R_mat{2,2, arma::fill::eye};
        R_mat *= R;

        // Ki = max_sizex2
        arma::mat Ki = (sigma_t*H.t())*((H*sigma_t*H.t() + R_mat).i());
        K_out = Ki;
        // Compute posterior state
        // Ki*(z_diff) = max_sizex1
        // belief = max_sizex1
        arma::mat z_calc = z - z_estimate;
        //z_calc(0) = turtlelib::normalize_angle(z_calc(0));
        belief_t = belief_t + Ki*(z_calc);
        //belief_t(0) = normalize_angle(belief_t(0));

        // Compute posterior covariance
        sigma_t = (arma::eye<arma::mat>(max_size, max_size) - Ki*H)*sigma_t;
        cov_out = sigma_t;
    }

    arma::colvec EKFSlam::get_qt(){
        q_t(0) = belief_t(0);
        q_t(1) = belief_t(1);
        q_t(2) = belief_t(2);
        return q_t;
        }

    arma::colvec EKFSlam::get_belief(){
        return belief_t;
        }

    arma::colvec EKFSlam::get_h(){
        return H_output;
        }

    arma::colvec EKFSlam::get_belief_predict(){
        return belief_t;
        }

    arma::colvec EKFSlam::get_mt_track(){
        return obstacle_tracking;
    }

}
