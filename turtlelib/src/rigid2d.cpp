#include "turtlelib/rigid2d.hpp"
#include <cstdio>
#include <iostream>
#include <limits.h>

namespace turtlelib{

    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]";
        return os; 
    }

    std::istream & operator>>(std::istream & is, Vector2D & v){
        if (is.peek() == '['){ // If the first character input is a bracket
            is.get();               // Remove bracket
            is >> v.x;              // Store first component in v.x
            is.get();               // Remove space
            is >> v.y;              // Store seecond component in v.y
        }
        else{
            is >> v.x >> v.y;
        }
        is.clear();
        is.ignore(INT_MAX, '\n');
        return is;
    }

    // Return idenetity matrix if no input given
    Transform2D::Transform2D() : trans_in{0.0, 0.0}, angle_in(0.0) {}

    // Create a pure translation rotation matrix
    Transform2D::Transform2D(Vector2D trans) : trans_in{trans}, angle_in(0.0) {}

    // Create a pure rotation rotation matrix
    Transform2D::Transform2D(double radians) : trans_in{0.0, 0.0}, angle_in(deg2rad(radians)) {}

    // Create a translation + rotation rotation matrix
    Transform2D::Transform2D(Vector2D trans, double radians) : trans_in{trans}, angle_in(deg2rad(radians)) {}

    // Perform a transformation on a Vector2D
    Vector2D Transform2D::operator()(Vector2D v) const{
        Vector2D new_vec={
                std::cos(angle_in)*v.x-std::sin(angle_in)*v.y + trans_in.x,
                std::sin(angle_in)*v.x+std::cos(angle_in)*v.y + trans_in.y
        };
        return new_vec;
    }

    // Perform a transformation on a Twist2D
    Twist2D Transform2D::operator()(Twist2D og_twist) const{
        Twist2D new_twist;

        new_twist.x=og_twist.w*trans_in.y+
                   std::cos(angle_in)*og_twist.x-
                   og_twist.y*std::sin(angle_in);

        new_twist.y = -og_twist.w*trans_in.x+
                      og_twist.x*std::sin(angle_in)+
                      og_twist.y*std::cos(angle_in);

        new_twist.w=og_twist.w;

        return new_twist;
    }

    // Return the inverse of a transformation matrix
    Transform2D Transform2D::inv() const{
        Transform2D inv_tf = {
            {std::cos(angle_in)*(-trans_in.x)-std::sin(angle_in)*trans_in.y,
             std::cos(angle_in)*(-trans_in.y)+std::sin(angle_in)*trans_in.x},
            -rad2deg(angle_in)
            };
            return inv_tf;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        // Modify the x value
        this->trans_in.x=std::cos(angle_in)*rhs.trans_in.x-
                         std::sin(angle_in)*rhs.trans_in.y +
                         this->trans_in.x;

        // Modify the y value
        this->trans_in.y=std::sin(angle_in)*rhs.trans_in.x+
                         std::cos(angle_in)*rhs.trans_in.y+
                         this->trans_in.y;
        return *this;
    }

    // Return the translational component of the TF matrix
    Vector2D Transform2D::translation() const{
        return trans_in;
    }

    // Return the rotational component of the TF matrix
    double Transform2D::rotation() const{
        return angle_in;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        os << "deg: " << rad2deg(tf.angle_in) << " x: " << tf.trans_in.x << " y: " << tf.trans_in.y;
        return os;
    }

    // Read a Transform2D as 3 numbers
    std::istream & operator>>(std::istream & is, Transform2D & tf){
        std::string tmp1, tmp2, tmp3;
        double deg;
        double x;
        double y;
        if (is.peek() == 'd'){ // If the first character input is a bracket
            is >> tmp1 >> deg >> tmp2 >> x >> tmp3 >> y;
        }
        else{
            is >> deg >> x >> y;
        }
        tf = Transform2D{{x,y},deg};
        is.clear();
        is.ignore(INT_MAX, '\n');
        return is;
    }

    // Multiply 2 matrices and return the output
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        Transform2D output = lhs;
        return output*=rhs;
    }

    std::ostream & operator<<(std::ostream & os, const Twist2D & twist){
        os << "[" << twist.w << " " << twist.x << " " << twist.y << "]";
        return os; 
    }

    std::istream & operator>>(std::istream & is, Twist2D & twist){
        if (is.peek() == '['){ // If the first character input is a bracket
            is.get();               // Remove bracket
            is >> twist.w >> twist.x >> twist.y;
        }
        else{
            is >> twist.w >> twist.x >> twist.y;
        }
        is.clear();
        is.ignore(INT_MAX, '\n');
        return is;
    }

    Vector2D Vector2D::normalize(){
        double mag = sqrt(x*x+y*y);
        return Vector2D{x/mag, y/mag};
    }

    double normalize_angle(double rad){ 
        // Modify the theta value 
        if (abs(rad) > PI){
            double norm_rad= fmod(rad, PI);
            return norm_rad;
        }
        else if(almost_equal(rad, -PI)){
            return PI;
        }
        else{return rad;}
    }
}