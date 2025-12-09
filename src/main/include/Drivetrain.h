#pragma once

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
//#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/AnalogGyro.h>

#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/Pigeon2.hpp>
#include "SwerveModule.h"

#include <choreo/trajectory/Trajectory.h>


namespace drivetrain
{
    // Constant Values for max bot linear velocity (Drive Speed) and angular velocity (Spin/turn Speed)

    static constexpr units::meters_per_second_t kMaxSpeed = 2.5_mps;  // 2.5 meters per second
    static constexpr units::radians_per_second_t kMaxAngularSpeed{ std::numbers::pi };  // 1/2 rotation per second 

    static constexpr units::meter_t kWheelDistance = 0.54782769475136_m;  // The distance bewteen 2 adjacent (like 1 edge of the square, not the diagnol) 
}

using namespace ctre::phoenix6;

class Drivetrain
{
public:
    // Constructor/Destructor.
    Drivetrain();
    ~Drivetrain() 
        { }

    // Accessor Methods.
    inline void ToggleFieldRelative()
        {  m_bIsFieldRelative = !m_bIsFieldRelative;  }

    inline bool IsFieldRelative()
        {  return {m_bIsFieldRelative};  }


    inline frc::Rotation2d GetGyroRotation2d()
        { return{ frc::Rotation2d(units::degree_t{ -m_gyro.GetYaw().GetValue() }) }; }

    // Class Methods.
    /**
     * Initializes the drivetrain class.
     * 
    */
    void Initialize ();

    /**
   * Function that takes in the 3 Joystick Axes and converts them to chassis speeds, then calls 
   * either DriveFieldRelative() or DriveBotRelative() dependent on m_bIsFieldRelative.
   * True - DriveFieldRelative()
   * False - DriveBotRelative()
   *
   * @param leftYJoystickValue The left Y joystick value of the driver controller.
   *     Range -1 to 1.
   * @param leftXJoystickValue The left X joystick value of the driver controller.
   *     Range -1 to 1.
   * @param rightXJoystickValue The right X joystick value of the driver controller.
   *     Range -1 to 1.
   */
    void Execute (  double leftYJoystickValue,          //X speed (forward/backward) - m/s
                    double leftXJoystickValue,          //Y speed (left/right) - m/s
                    double rightXJoystickValue  );      //Rotation Speed - rad/s

    /**
   * Function that drives the robot field-relative, converting inputted speeds into field relative speeds
   * (rotating the vectors) to receive the desired output direction. This is either automatically called by
   * Execute, or can be manually called to bypass all driver speed limiters, slew rate limiters, and the
   * field vs. bot relative boolean 
   *
   * @param xSpeed The X speed of the bot in a field speed (+ is towards opposing driver stations, - is towards own driver station). 
   * Units: meters per second
   * @param ySpeed The Y speed of the bot in a field speed (+ is left from driver perspective, - is right from driver perspective).
   * Units: meters per second
   * @param rotSpeed The rotational speed of the bot (+ is Counter-Clockwise, - is Clockwise)
   * Units: radians per second
   */
    void DriveFieldRelative( double xSpeed, double ySpeed, double rotSpeed );
    
    /**
   * Function that drives the robot bot-relative, inputted speeds stay in respective directions.
   * This is either automatically called by Execute, or can be manually called to bypass all 
   * driver speed limiters, slew rate limiters, and the field vs. bot relative boolean 
   *
   * @param xSpeed The X speed of the bot in a bot speed (+ is towards robot front/forwards, - is towards robot back/backwards). 
   * Units: meters per second
   * @param ySpeed The Y speed of the bot in a bot speed (+ is robot-perspective left, - is robot-perspective right). 
   * Units: meters per second
   * @param rotSpeed The rotational speed of the bot (+ is Counter-Clockwise, - is Clockwise)
   * Units: radians per second
   */
    void DriveBotRelative( double xSpeed, double ySpeed, double rotSpeed );

    void Stop();

    SwerveModule m_frontLeft{1,2,9};
    SwerveModule m_frontRight{3,4,10};
    SwerveModule m_backLeft{5,6,11};
    SwerveModule m_backRight{7,8,12};

private:
    hardware::Pigeon2 m_gyro{13};

    bool m_bIsFieldRelative;    //Field Relative bool (true: field centric; false: bot centric)

    bool m_bLockOnStop; //Lock on stop bool - turn wheels to 45s

    double m_dGyroOffset;

    frc::Translation2d m_frontLeftLocation{+drivetrain::kWheelDistance / 2, -drivetrain::kWheelDistance / 2};     //Front Left Wheel Position
    frc::Translation2d m_frontRightLocation{+drivetrain::kWheelDistance / 2, +drivetrain::kWheelDistance / 2};    //Front Right Wheel Position
    frc::Translation2d m_backLeftLocation{-drivetrain::kWheelDistance / 2, -drivetrain::kWheelDistance / 2};      //Back Left Wheel Position
    frc::Translation2d m_backRightLocation{-drivetrain::kWheelDistance / 2, +drivetrain::kWheelDistance / 2};     //Back Right Wheel Position

    frc::SlewRateLimiter<units::scalar> m_xSpeedLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_ySpeedLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_rotSpeedLimiter{3 / 1_s};

    // This is an object that is used to go from Bot Speed Values (X,Y,Rot) to 
    // 4 different swerve module states (Turn angle and Speed).
    frc::SwerveDriveKinematics<4> m_kinematics{
        m_frontLeftLocation,    
        m_frontRightLocation,
        m_backLeftLocation,
        m_backRightLocation
    };
};