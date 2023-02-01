#include "turtlelib/diff_drive.hpp"

namespace turtlelib{

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

        //phi.r = (l/2*theta+Vb.x)/r
        phi.r = ((this->track_length/2)*Vb.w+Vb.x)/(this->wheel_radius);

        //phi.l = (Vb.x-theta*l/2)/r
        phi.l = (Vb.x - (this->track_length/2)*Vb.w)/(this->wheel_radius);

        return phi;
    }

}