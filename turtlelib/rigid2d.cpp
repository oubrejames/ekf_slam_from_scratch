#include "rigid2d.hpp"
#include <cstdio>
#include <cmath>
#include <iostream>


/// \brief output a 2 dimensional vector as [xcomponent ycomponent]
/// os - stream to output to
/// v - the vector to print
std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Vector2D & v){
    os << "[" << v.x << " " << v.y << "]";
    /// \TODO: Why do we return OS?
    return os; 
}

/// \brief input a 2 dimensional vector
///   You should be able to read vectors entered as follows:
///   [x y] or x y
/// \param is - stream from which to read
/// \param v [out] - output vector
std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Vector2D & v){
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

/// \brief a rigid body transformation in 2 dimensions
class Transform2D
{
public:
    /// \brief Create an identity transformation
    Transform2D();

    /// \brief create a transformation that is a pure translation
    /// \param trans - the vector by which to translate
    explicit Transform2D(Vector2D trans);

    /// \brief create a pure rotation
    /// \param radians - angle of the rotation, in radians
    explicit Transform2D(double radians);

    /// \brief Create a transformation with a translational and rotational
    /// component
    /// \param trans - the translation
    /// \param rot - the rotation, in radians
    Transform2D(Vector2D trans, double radians);

    /// \brief apply a transformation to a Vector2D
    /// \param v - the vector to transform
    /// \return a vector in the new coordinate system
    Vector2D operator()(Vector2D v) const;


    /// \brief invert the transformation
    /// \return the inverse transformation. 
    Transform2D inv() const;

    /// \brief compose this transform with another and store the result 
    /// in this object
    /// \param rhs - the first transform to apply
    /// \return a reference to the newly transformed operator
    Transform2D & operator*=(const Transform2D & rhs);

    /// \brief the translational component of the transform
    /// \return the x,y translation
    Vector2D translation() const;

    /// \brief get the angular displacement of the transform
    /// \return the angular displacement, in radians
    double rotation() const;

    /// \brief \see operator<<(...) (declared outside this class)
    /// for a description
    friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

};


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

    return 0;
}