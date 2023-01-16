#include "rigid2d.hpp"
#include <cstdio>
#include <iostream>
#include <limits.h>

namespace turtlelib{
    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]";
        /// \TODO: Why do we return OS?
        return os; 
    }

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param v [out] - output vector
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
    Transform2D::Transform2D(double radians) : trans_in{0.0, 0.0}, angle_in(radians) {}

    // Create a translation + rotation rotation matrix
    Transform2D::Transform2D(Vector2D trans, double radians) : trans_in{trans}, angle_in(radians) {}

    /// \TODO: ADD COMMENT
    Vector2D Transform2D::operator()(Vector2D v) const{
        Vector2D new_vec={
                std::cos(angle_in)*v.x-std::sin(angle_in)*v.y + trans_in.x,
                std::sin(angle_in)*v.x+std::cos(angle_in)*v.y + trans_in.y
        };
        return new_vec;
    }

    /// \TODO: ADD COMMENT
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

    /// \TODO: ADD COMMENT
    Transform2D Transform2D::inv() const{
        Transform2D inv_tf = {
            {std::cos(angle_in)*(-trans_in.x)-std::sin(angle_in)*trans_in.y,
             std::cos(angle_in)*(-trans_in.y)+std::sin(angle_in)*trans_in.x},
            -angle_in
            };
            return inv_tf;
    }

    /// \TODO: ADD COMMENT
    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        // Modify the x value
        this->trans_in.x=std::cos(angle_in)*rhs.trans_in.x-
                         std::sin(angle_in)*rhs.trans_in.y +
                         this->trans_in.x;

        // Modify the y value
        this->trans_in.y=std::sin(angle_in)*rhs.trans_in.x+
                         std::cos(angle_in)*rhs.trans_in.y+
                         this->trans_in.y;

        // Modify the theta value 
        /// \test
        this->angle_in=std::acos(-std::sin(angle_in)*std::sin(rhs.angle_in)+
                        std::cos(angle_in)*std::cos(rhs.angle_in));
        return *this;
    }

    // Return the translational component of the TF matrix
    Vector2D Transform2D::translation() const{
        /// TODO: is this right? should I actually perform a translation
        return trans_in;
    }

    // Return the rotational component of the TF matrix
    double Transform2D::rotation() const{
        /// TODO: is this right? should I actually perform some operation
        return rad2deg(angle_in);
    }

    /// \TODO: ADD COMMENT
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        os << "deg: " << rad2deg(tf.angle_in) << " x: " << tf.trans_in.x << " y: " << tf.trans_in.y;
        return os;
    }

    // Read a Transform2D as 3 numbers
    std::istream & operator>>(std::istream & is, Transform2D & tf){
        // & tf refers to a transform2D object, if you change the variable here, it will change the
        // contents at that memory location 
        // we are reading in the user input from the is stream via is >> output
        // we can then make changes to the tf variable here to change it in memory
        
        ///TODO: read in as output from operator<<
            ///TODO: read cppreference.com on istream
            ///TODO: make multiple istream things work
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
        tf = Transform2D{{x,y},deg2rad(deg)};
        is.clear();
        is.ignore(INT_MAX, '\n');
        return is;
    }

    // Multiply 2 matrices and return the output
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        Transform2D output = lhs; // use output variable as to not change either argument after the operation
        return output*=rhs;
    }

    std::ostream & operator<<(std::ostream & os, const Twist2D & twist){
        os << "[" << twist.w << " " << twist.x << " " << twist.y << "]";
        return os; 
    }

    /// \TODO: doxygen
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

    /// \TODO: comment
    Vector2D Vector2D::normalize(){
        double mag = sqrt(x*x+y*y);
        return Vector2D{x/mag, y/mag};
    }
}