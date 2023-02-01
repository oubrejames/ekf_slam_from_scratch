#include "turtlelib/diff_drive.hpp"

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

    Twist2D DiffDrive::foward_kinematics(const WheelPos u) const{
        // Calculate the angular portion of the twist
        // [(u.r/4 - uy.l/4)*r^2]/l
        double w = (0.25*(u.r-u.l)*(this->wheel_radius*this->wheel_radius))/(0.5*this->track_length);

        // Calculate the x portion of the twist
        // (u.r/4 + uy.l/4)*r^2
        double x = 0.25*(u.r+u.l)*(this->wheel_radius*this->wheel_radius);

        // y portion of twist = 0
        Twist2D body_twist = {{x,0},w};
        return body_twist
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

}