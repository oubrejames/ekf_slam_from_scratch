#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP

#include "rigid2d.hpp"

namespace turtlelib
{
    /// @brief the position (phi) of the robot's left and right wheels
    struct WheelPos{
        // Position of right wheel
        double r = 0.0;

        // Position of left wheel
        double l =0.0;
    }

    /// @brief the configuration (q) of the robot in the world frame
    struct RobotConfig{
        // x position of the robot
        double x = 0;

        // y position of the robot
        double y = 0;

        // theta position of the robot
        double theta = 0;
    }

    /// \brief kinematics of a diff drive robot
    class DiffDrive
    {
    private:
        double track_length;
        double wheel_radius;
        RobotConfig current_pos;
        WheelPos current_wheel_pos;

    public:
        /// @brief Given a wheel position command calculate the body twist of the robot
        /// @param u the commanded wheel position
        /// @return a Twist2D that is the body twist of the robot
        Twist2D foward_kinematics(const WheelPos u) const;

        /// TODO: Questions!!
        // making wheel velocity = displacement cause delta t = 1 ?
        // Made const because it does not change any existing variables
        // Also made Vb a const cause it will never change
        /// @brief given a body twist, compute the wheel velocities necessary to achieve it
        /// @param Vb the body twist acting on the robot
        /// @return the corresponding wheel velocities 
        WheelPos inverse_kinematics(const Twist2D Vb) const;

    }
}
#endif
