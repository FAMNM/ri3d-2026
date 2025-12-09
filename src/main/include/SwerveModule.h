#pragma once

#include <numbers>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <frc/Timer.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>


using namespace ctre::phoenix6;

namespace swerveModule
{
    static constexpr double kWheelRadius = 0.0508;  //Wheel Radius in Meters
    static constexpr double kDriveGearRatio = 7.85;  //Drive motor to wheel gear ratio
    static constexpr double kTurnGearRatio = 12.1;
}

class SwerveModule
{
public:
    /**
     * Swerve module constructor. Create 1 for each swerve module on bot, report relative locations in Drivetrain.h.
     * 
     * @param driveID The CAN ID for the drive motor.
     * 
     * @param turnID The CAN ID for the turning motor.
     * 
     * @param encoderID The CAN ID for the absolute encoder used to read wheel angle.
    */
    SwerveModule(const int driveID, const int turnID, const int encoderID);
    ~SwerveModule()
        {  }

    // Accessor Methods.

    /**
     * Gets the current state of the Swerve module.
     * 
     * @returns SwerveModuleState object with an angle for the current wheel (Rotation2d)
     *      and the speed of the wheel (m/s)
    */
    frc::SwerveModuleState GetState();

    /**
     * Gets the current position of the Swerve module.
     * 
     * @returns SwerveModulePosition object with an angle for the current wheel (Rotation2d)
     *      and the distance the wheel traveld (m)
    */
    frc::SwerveModulePosition GetPosition();
  
    /**
     * Sets the state for the Swerve Module. This is the main function for driving the module
     * 
     * @param state The Swerve Module State. Contains a desired speed (m/s) and an angle (rad).
     *          This can be made manually or output by the SwerveDriveKinematics class.
    */
    void SetDesiredState(const frc::SwerveModuleState& state);

    /**
     * Configs the motors and encoders in the Swerve Module. This is where all the current limits,
     * break modes, and inverted modes can be set.
    */
    void ConfigModule();
    void ResetDriveEncoder();
    void Stop();

private:
    hardware::TalonFX m_driveMotor;
    hardware::TalonFX m_turningMotor;
    hardware::CANcoder m_turningEncoder;

    controls::VelocityVoltage m_request{0_tps};
    controls::MotionMagicVoltage m_turnRequest{0_tr};
};