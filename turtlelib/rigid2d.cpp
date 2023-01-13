#include "rigid2d.hpp"
#include <cstdio>
#include <cmath>
#include <iostream>

using turtlelib::Vector2D;
using turtlelib::Transform2D;

/// \brief output a 2 dimensional vector as [xcomponent ycomponent]
/// os - stream to output to
/// v - the vector to print
std::ostream & turtlelib::operator<<(std::ostream & os, const Vector2D & v){
    os << "[" << v.x << " " << v.y << "]";
    /// \TODO: Why do we return OS?
    return os; 
}

/// \brief input a 2 dimensional vector
///   You should be able to read vectors entered as follows:
///   [x y] or x y
/// \param is - stream from which to read
/// \param v [out] - output vector
std::istream & turtlelib::operator>>(std::istream & is, Vector2D & v){
    if (is.peek() == '['){ // If the first character input is a bracket
        is.get();               // Remove bracket
        is >> v.x;              // Store first component in v.x
        is.get();               // Remove space
        is >> v.y;              // Store seecond component in v.y
    }
    else{
        is >> v.x >> v.y;
    }
    return is;
}

namespace turtlelib{
    // Return idenetity matrix if no input given
    Transform2D::Transform2D() : trans_in{0.0, 0.0}, radians_in(0.0) {}

    // Create a pure translation rotation matrix
    Transform2D::Transform2D(Vector2D trans) : trans_in{trans}, radians_in(0.0) {}

    // Create a pure rotation rotation matrix
    Transform2D::Transform2D(double radians) : trans_in{0.0, 0.0}, radians_in(radians) {}

    // Create a translation + rotation rotation matrix
    Transform2D::Transform2D(Vector2D trans, double radians) : trans_in{trans}, radians_in(radians) {}

    Vector2D Transform2D::operator()(Vector2D v) const{
        Vector2D new_vec={
                std::cos(radians_in)*v.x-std::sin(radians_in)*v.y+v.x,
                std::cos(radians_in)*v.y+std::sin(radians_in)*v.x+v.y
        };
        return new_vec;
    }

    Transform2D Transform2D::inv() const{
        Transform2D inv_tf = {
            {std::cos(radians_in)*(-trans_in.x)-std::sin(radians_in)*trans_in.y,
             std::cos(radians_in)*(-trans_in.y)+std::sin(radians_in)*trans_in.x},
            -radians_in
            };
            return inv_tf;
    }

    /// \brief compose this transform with another and store the result 
    /// in this object
    /// \param rhs - the first transform to apply
    /// \return a reference to the newly transformed operator
    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        // Modify the x value
        this->trans_in.x=std::cos(this->radians_in)*rhs.trans_in.x-
                         std::sin(radians_in)*rhs.trans_in.y;

        // Modify the y value
        this->trans_in.y=std::cos(this->radians_in)*rhs.trans_in.x-
                         std::sin(radians_in)*rhs.trans_in.y;

        // Modify the theta value 
        this->radians_in=std::acos(-std::sin(this->radians_in)*std::sin(rhs.radians_in)+
                        std::cos(this->radians_in)*std::cos(rhs.radians_in));
    }
}
// /// \brief should print a human readable version of the transform:
// /// An example output:
// /// deg: 90 x: 3 y: 5
// /// \param os - an output stream
// /// \param tf - the transform to print
// std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

// /// \brief Read a transformation from stdin
// /// Should be able to read input either as output by operator<< or
// /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
// /// For example:
// /// 90 2 3
// std::istream & operator>>(std::istream & is, Transform2D & tf);

// /// \brief multiply two transforms together, returning their composition
// /// \param lhs - the left hand operand
// /// \param rhs - the right hand operand
// /// \return the composition of the two transforms
// /// HINT: This function should be implemented in terms of *=
// Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

int main() {
    // Variable declarations
    double x=1.2;
    double y=4.5;

    // Program code
    int i = turtlelib::almost_equal(x,y);
    printf("%d\n", i);
    printf("Deg to rads: %f\n",turtlelib::deg2rad(90.0));
    printf("Rad to deg: %f\n",turtlelib::rad2deg(3.14));

    turtlelib::Vector2D c1;
    std::cout << "Enter a Vector2D: ";
    std::cin >> c1;
    std::cout << "x = " << c1.x << " y=" << c1.y << std::endl;
    std::cout << c1 << std::endl;

    Transform2D test_tf = {c1, -9.5};
    Transform2D inv = test_tf.inv();
    // std::cout << "Inverse" << inv.tr << std::endl;
    return 0;
}