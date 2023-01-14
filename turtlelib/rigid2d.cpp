#include "rigid2d.hpp"
#include <cstdio>
#include <cmath>
#include <iostream>

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
        return is;
    }

    // Return idenetity matrix if no input given
    Transform2D::Transform2D() : trans_in{0.0, 0.0}, radians_in(0.0) {}

    // Create a pure translation rotation matrix
    Transform2D::Transform2D(Vector2D trans) : trans_in{trans}, radians_in(0.0) {}

    // Create a pure rotation rotation matrix
    Transform2D::Transform2D(double radians) : trans_in{0.0, 0.0}, radians_in(radians) {}

    // Create a translation + rotation rotation matrix
    Transform2D::Transform2D(Vector2D trans, double radians) : trans_in{trans}, radians_in(radians) {}

    /// \TODO: ADD COMMENT
    Vector2D Transform2D::operator()(Vector2D v) const{
        Vector2D new_vec={
                std::cos(radians_in)*v.x-std::sin(radians_in)*v.y+v.x,
                std::cos(radians_in)*v.y+std::sin(radians_in)*v.x+v.y
        };
        return new_vec;
    }

    /// \TODO: ADD COMMENT
    Transform2D Transform2D::inv() const{
        Transform2D inv_tf = {
            {std::cos(radians_in)*(-trans_in.x)-std::sin(radians_in)*trans_in.y,
             std::cos(radians_in)*(-trans_in.y)+std::sin(radians_in)*trans_in.x},
            -radians_in
            };
            return inv_tf;
    }

    /// \TODO: ADD COMMENT
    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        // Modify the x value
        this->trans_in.x=std::cos(this->radians_in)*rhs.trans_in.x-
                         std::sin(radians_in)*rhs.trans_in.y +
                         this->trans_in.x;

        // Modify the y value
        this->trans_in.y=std::sin(this->radians_in)*rhs.trans_in.x+
                         std::cos(radians_in)*rhs.trans_in.y+
                         this->trans_in.y;

        // Modify the theta value 
        this->radians_in=std::acos(-std::sin(this->radians_in)*std::sin(rhs.radians_in)+
                        std::cos(this->radians_in)*std::cos(rhs.radians_in));
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
        return radians_in;
    }


    /// \TODO: ADD COMMENT
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        os << "deg: " << tf.radians_in << " x: " << tf.trans_in.x << " y: " << tf.trans_in.y;
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
        tf = Transform2D{{x,y}, deg};
        
        return is;
    }

    // Multiply 2 matrices and return the output
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        Transform2D output = lhs; // use output variable as to not change either argument after the operation
        return output*=rhs;
    }
}

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

    turtlelib::Transform2D test_tf = {c1, -9.5};
    turtlelib::Transform2D inv = test_tf.inv();
    std::cout << "Inverse " << inv << std::endl;

    turtlelib::Transform2D test = {{4,5},9.0};
    std::cout << "Testing transform2d cout: " << test << std::endl;

    //std::cin.sync();
    std::cout << "Enter Transform2d: ";
    turtlelib::Transform2D tf;
    std::cin >> tf;
    std::cout << "The transformation matrix: " << tf << std::endl;
    return 0;
}