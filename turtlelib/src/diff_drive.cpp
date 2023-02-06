#include "turtlelib/diff_drive.hpp"
#include <stdexcept>
#include <iostream>
namespace turtlelib{

    DiffDrive::DiffDrive(double track_l, double wheel_r, RobotConfig pos, WheelPos w_pos) : 
            track_length{track_l},
            wheel_radius{wheel_r},
            current_pos{pos},
            current_wheel_pos{w_pos} {}

    DiffDrive::DiffDrive(double track_l, double wheel_r) : 
            track_length{track_l},
            wheel_radius{wheel_r},
            current_pos{0.0, 0.0, 0.0},
            current_wheel_pos{0.0, 0.0} {}
    

    DiffDrive::DiffDrive(double track_l, double wheel_r, WheelPos w_pos) : 
            track_length{track_l},
            wheel_radius{wheel_r},
            current_pos{0.0, 0.0, 0.0},
            current_wheel_pos{w_pos} {}

    DiffDrive::DiffDrive(double track_l, double wheel_r, RobotConfig pos) : 
            track_length{track_l},
            wheel_radius{wheel_r},
            current_pos{pos},
            current_wheel_pos{0.0, 0.0} {}

    /// TODO: should this update? should this return
    /// if absolute value = u must subtract (current wheel pos - u)
    // if u is relative, we good
    Twist2D DiffDrive::forward_kinematics(const WheelPos u) {
        // Calculate the angular portion of the twist
        // [(u.r/4 - uy.l/4)*r^2]/l
        double w = ((u.r-u.l)*(this->wheel_radius))/(this->track_length);

        // Calculate the x portion of the twist
        // (u.r/4 + uy.l/4)*r^2
        double x = 0.5*(u.r+u.l);

        // y portion of twist = 0
        Twist2D body_twist = {w, x, 0.0};
        update_robot_pos(body_twist);
        return body_twist;
    }

    /// TODO: What angle is theta here
    /// should i be adding on to my current position
    void DiffDrive::update_robot_pos(const Twist2D Vb){
        // Integrate body twist
        Transform2D Tbb = integrate_twist(Vb);
        double delta_theta = Tbb.rotation();
        Vector2D delta_tran = Tbb.translation();

        current_pos.x += cos(current_pos.theta)*delta_tran.x-sin(current_pos.theta)*delta_tran.y;
        current_pos.y += sin(current_pos.theta)*delta_tran.x+cos(current_pos.theta)*delta_tran.y;
        current_pos.theta = delta_theta;
    }

    WheelPos DiffDrive::inverse_kinematics(const Twist2D Vb) const{
        WheelPos phi;

        if(almost_equal(Vb.y, 0.0)){
            //phi.r = (l/2*theta+Vb.x)/r
            phi.r = ((this->track_length/2)*Vb.w+Vb.x)/(this->wheel_radius);

            //phi.l = (Vb.x-theta*l/2)/r
            phi.l = (Vb.x - (this->track_length/2)*Vb.w)/(this->wheel_radius);
        }
        else {
            throw std::logic_error("Given body twist causes slipping.");
        }
        return phi;
    }

    RobotConfig DiffDrive::get_current_pos() const{
        return current_pos;
    }

    WheelPos DiffDrive::get_current_wheel_pos() const{
        return current_wheel_pos;
    }
}