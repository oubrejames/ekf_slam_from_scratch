#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP

#include "rigid2d.hpp"

namespace turtlelib
{
    /// @brief the position (phi) of the robot's left and right wheels
    struct WheelPos{
         /// @brief Position of right wheel
        double r = 0.0;

        /// @brief Position of left wheel
        double l =0.0;
    };

    /// @brief the configuration (q) of the robot in the world frame
    struct RobotConfig{
        /// @brief x position of the robot
        double x = 0;

        /// @brief y position of the robot
        double y = 0;

        /// @brief theta position of the robot
        double theta = 0;
    };

    /// \brief kinematics of a diff drive robot
    class DiffDrive
    {
    private:
        double track_length;
        double wheel_radius;
        RobotConfig current_pos;
        WheelPos current_wheel_pos;

    public:
        /// @brief Create a diff drive object with defined track length, wheel radius, robot position
        /// and wheel position
        /// @param track_l the length between wheels
        /// @param wheel_r the radus of the robot's wheels
        /// @param pos the position of the  robot
        /// @param w_pos the position of the robots wheels
        DiffDrive(double track_l, double wheel_r, RobotConfig pos, WheelPos w_pos);

        /// @brief Create a diff drive object with just a defined track length and wheel radius
        /// @param track_l the length between wheels
        /// @param wheel_r the radus of the robot's wheels
        DiffDrive(double track_l, double wheel_r);

        /// @brief Create a diff drive object with defined track length, wheel radius, and wheel position
        /// @param track_l the length between wheels
        /// @param wheel_r the radus of the robot's wheels
        /// @param w_pos the position of the robots wheels
        DiffDrive(double track_l, double wheel_r, WheelPos w_pos);

        /// @brief Create a diff drive object with defined track length, wheel radius, and robot position
        /// @param track_l the length between wheels
        /// @param wheel_r the radus of the robot's wheels
        /// @param pos the position of the  robot
        DiffDrive(double track_l, double wheel_r, RobotConfig pos);

        /// @brief Given a wheel position command calculate the body twist of the robot
        /// @param u the commanded wheel position (delta phi)
        /// @return a Twist2D that is the body twist of the robot
        Twist2D forward_kinematics(const WheelPos u);

        /// @brief Update the configuration of the robot in the robot frame 
        /// @param Vb the commanded body twist
        void update_robot_pos(const Twist2D Vb);

        /// TODO: Questions!!
        // making wheel velocity = displacement cause delta t = 1 ?
        // Made const because it does not change any existing variables
        // Also made Vb a const cause it will never change
        /// @brief given a body twist, compute the wheel velocities necessary to achieve it
        /// @param Vb the body twist acting on the robot
        /// @return the corresponding wheel velocities 
        WheelPos inverse_kinematics(const Twist2D Vb) const;

        /// \brief get the current position of the robot
        /// \return the robot's current position as a RobotConfig
        RobotConfig get_current_pos() const;

        /// \brief get the current wheel position of the robot
        /// \return the robot's current wheel position as a WheelPos
        WheelPos get_current_wheel_pos() const;
    };
}
#endif
