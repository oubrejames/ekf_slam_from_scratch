#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>

namespace turtlelib
{
/// \brief PI.  Not in C++ standard until C++20.
constexpr double PI = 3.14159265358979323846;

/// \brief approximately compare two floating-point numbers using
///        an absolute comparison
/// \param d1 - a number to compare
/// \param d2 - a second number to compare
/// \param epsilon - absolute threshold required for equality
/// \return true if abs(d1 - d2) < epsilon
/// NOTE: implement this in the header file
/// constexpr means that the function can be computed at compile time
/// if given a compile-time constant as input
constexpr bool almost_equal(double d1, double d2, double epsilon = 1.0e-12)
{
  return abs(d1 - d2) < epsilon;
}

/// \brief convert degrees to radians
/// \param deg - angle in degrees
/// \returns radians
constexpr double deg2rad(double deg)
{
  return deg * (PI / 180.0);
}

/// \brief convert radians to degrees
/// \param rad - angle in radians
/// \returns the angle in degrees
constexpr double rad2deg(double rad)
{
  return rad * (180.0 / PI);
}

static_assert(almost_equal(0, 0), "is_zero failed");

static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

/// \brief A 2-Dimensional Vector [x,y]
struct Vector2D
{
  /// \brief the x coordinate
  double x = 0.0;

  /// \brief the y coordinate
  double y = 0.0;

  /// \brief normalize a Vector2D
  /// \return the unit vector of a Vector2D
  Vector2D normalize();

  /// \brief multiply this Vector2D with a scalar and store the result
  /// in this object
  /// \param rhs - the first Vector to apply
  /// \return a reference to the newly transformed operator
  Vector2D & operator*=(const double rhs);

  /// \brief subtract a Vector2D from this Vector2D and store the result in this object
  /// \param rhs - the first Vector to apply
  /// \return a reference to the newly transformed operator
  Vector2D & operator-=(const Vector2D & rhs);

  /// \brief add this Vector2D with another and store the result in this object
  /// \param rhs - the first Vector to apply
  /// \return a reference to the newly transformed operator
  Vector2D & operator+=(const Vector2D & rhs);

  /// @brief obtain the magnitude of a Vector2D
  /// @return the vector's magnitude
  double magnitude();

};

/// @brief get the dot product between two Vector2D
/// @param lhs_vect the Vector2D to perform dot product with
/// @param rhs_vect the Vector2D to perform dot product with
/// @return the scalar product of the vectors
double dot(Vector2D lhs_vect, Vector2D rhs_vect);

/// @brief compute the angle between the current vector and another
/// @param lhs_vect one of the vectors to find the angle between
/// @param rhs_vect one of the vectors to find the angle between
/// @return the angle between vectors
double angle(Vector2D lhs_vect, Vector2D rhs_vect);

/// \brief A 2-Dimensional Twist [w,x,y]
struct Twist2D
{
  /// \brief rotational component
  double w = 0.0;

  /// \brief the x coordinate
  double x = 0.0;

  /// \brief the y coordinate
  double y = 0.0;
};

/// \brief output a 2 dimensional vector as [xcomponent ycomponent]
/// os - stream to output to
/// v - the vector to print
std::ostream & operator<<(std::ostream & os, const Vector2D & v);

/// \brief input a 2 dimensional vector as [x y] or x y
/// \param is - stream from which to read
/// \param v [out] - output vector
std::istream & operator>>(std::istream & is, Vector2D & v);

/// \brief a rigid body transformation in 2 dimensions
class Transform2D
{
private:
  Vector2D trans_in;
  double angle_in;

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
  /// \param radians - the rotation, in radians
  Transform2D(Vector2D trans, double radians);

  /// \brief apply a transformation to a Vector2D
  /// \param v - the vector to transform
  /// \return a vector in the new coordinate system
  Vector2D operator()(Vector2D v) const;

  /// \brief apply a transformation to a 2D twist
  /// \param og_twist - the twist to be transformed
  /// \return the twist in the updated frame
  Twist2D operator()(Twist2D og_twist) const;

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


/// \brief print a human readable version of the transform:
/// An example output:
/// deg: 90 x: 3 y: 5
/// \param os - an output stream
/// \param tf - the transform to print
std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

/// \brief Read a transformation from stdin
/// Read input either as output by operator<< or
/// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
/// For example:
/// 90 2 3
/// \param is - an input stream
/// \param tf - the transform to read
std::istream & operator>>(std::istream & is, Transform2D & tf);

/// \brief multiply two transforms together, returning their composition
/// \param lhs - the left hand operand
/// \param rhs - the right hand operand
/// \return the composition of the two transforms
Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

/// \brief print a human readable version of the transform:
/// An example output:
/// deg: 90 x: 3 y: 5
/// \param os - an output stream
/// \param twist - the transform to print
std::ostream & operator<<(std::ostream & os, const Twist2D & twist);

/// \brief Read a Twist2D from stdin
/// \param is - an input stream
/// \param twist - the twist to read
std::istream & operator>>(std::istream & is, Twist2D & twist);

/// @brief Normalize an angle in radians to be between -pi and pi
/// @param rad input angle
/// @return angle between -pi and pi
double normalize_angle(double rad);

/// \brief multiply two Vector2Ds together, returning their product
/// \param lhs - the left hand operand
/// \param rhs - the right hand operand
/// \return the product of the two vectors
Vector2D operator*(Vector2D lhs, const double rhs);

/// \brief multiply two Vector2Ds together, returning their product
/// \param lhs - the left hand operand
/// \param rhs - the right hand operand
/// \return the product of the two vectors
Vector2D operator*(const double lhs, Vector2D rhs);

/// \brief add two Vector2Ds together, returning their sum
/// \param lhs - the left hand operand
/// \param rhs - the right hand operand
/// \return the sum of the two vectors
Vector2D operator+(Vector2D lhs, const Vector2D & rhs);

/// \brief subtract one Vector2D from another, returning their difference
/// \param lhs - the left hand operand
/// \param rhs - the right hand operand
/// \return the difference of the two vectors
Vector2D operator-(Vector2D lhs, const Vector2D & rhs);

/// @brief compute the transformation corresponding to a rigid body following a constant twist
/// for one unit of time
/// @param  twist the twist to integrate
/// @return the Transform2D resulting from the integrated twist
Transform2D integrate_twist(Twist2D twist);
}

#endif
