#include <iostream>
#include "rigid2d.hpp"
#include <limits.h>

using turtlelib::Transform2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;

int main() {
    std::cout <<"Enter transform T_{a,b}:" << std::endl;
    Transform2D tab;
    std::cin >> tab;

    // Flush the istream to cin multiple times
    std::cin.clear();
    std::cin.ignore(INT_MAX, '\n');

    std::cout <<"Enter transform T_{b,c}:" << std::endl;
    Transform2D tbc;
    std::cin >> tbc;

    // Calclate Tba
    Transform2D tba = tab.inv();
    // Calclate Tcb
    Transform2D tcb = tbc.inv();
    // Calclate Tac
    Transform2D tac = tab*tbc;
    // Calclate Tca
    Transform2D tca = tcb*tba;

    // Display the transformation matricies values
    std::cout << "T_{a,b}: " << tab << std::endl;
    std::cout << "T_{b,a}: " << tba << std::endl;
    std::cout << "T_{b,c}: " << tbc << std::endl;
    std::cout << "T_{c,b}: " << tcb << std::endl;
    std::cout << "T_{a,c}: " << tac << std::endl;
    std::cout << "T_{c,a}: " << tca << std::endl;

    // Flush the istream to cin multiple times
    std::cin.clear();
    std::cin.ignore(INT_MAX, '\n');

    std::cout <<"Enter vector v_b:" << std::endl;
    Vector2D vb;
    std::cin >> vb;

    // Normalize
    Vector2D v_bhat = vb.normalize();

    // Get vb in a frame
    Vector2D va = tab(vb);
    // Get vb in c frame
    Vector2D vc = tcb(vb);

    // Display the transformed vectors
    std::cout << "v_bhat: " << v_bhat << std::endl;
    std::cout << "v_a: " << va << std::endl;
    std::cout << "v_b: " << vb << std::endl;
    std::cout << "v_c: " << vc << std::endl;

    // Flush the istream to cin multiple times
    std::cin.clear();
    std::cin.ignore(INT_MAX, '\n');

    std::cout <<"Enter twist V_b:" << std::endl;
    Twist2D vb_twist;
    std::cin >> vb_twist;

    // Get vb in a frame
    Twist2D va_twist = tab.switch_twist_frame(vb_twist);
    // Get vb in c frame
    Twist2D vc_twist = tcb.switch_twist_frame(vb_twist);

    // Display the transformed twists
    std::cout << "V_a: " << va_twist << std::endl;
    std::cout << "V_b: " << vb_twist << std::endl;
    std::cout << "V_c: " << vc_twist << std::endl;

    return 0;
}