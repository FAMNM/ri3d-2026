#include "SwerveModule.h"

#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(const int driveID,            //3 Parameters passed in when making class
                           const int turnID,
                           const int encoderID
                           )
    
    //Set all the passed in motor and encoder values

    : m_driveMotor(driveID),             //Set the objects to corresponding can IDs
      m_turningMotor(turnID),
      m_turningEncoder(encoderID)
{

}

void SwerveModule::ConfigModule()
{
    //Factory Default Configs
    configs::TalonFXConfiguration driveConfig = configs::TalonFXConfiguration{};
    configs::TalonFXConfiguration turnConfig = configs::TalonFXConfiguration{};

    driveConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RotorSensor;
    driveConfig.Feedback.RotorToSensorRatio = 1.0;
    driveConfig.Feedback.SensorToMechanismRatio = swerveModule::kDriveGearRatio;
  
    driveConfig.CurrentLimits.WithStatorCurrentLimit(40_A);
    driveConfig.CurrentLimits.WithSupplyCurrentLimit(40_A);
    driveConfig.CurrentLimits.WithStatorCurrentLimitEnable(true);
    driveConfig.CurrentLimits.WithSupplyCurrentLimitEnable(true);

    turnConfig.CurrentLimits.WithStatorCurrentLimit(40_A);
    turnConfig.CurrentLimits.WithSupplyCurrentLimit(40_A);
    turnConfig.CurrentLimits.WithStatorCurrentLimitEnable(true);
    turnConfig.CurrentLimits.WithSupplyCurrentLimitEnable(true);

    //Drive Onboarded PID Configs
    configs::Slot0Configs driveSlot0Configs{};
    driveSlot0Configs.kS = 0.21; //Output voltage to overcome static friction
    driveSlot0Configs.kV = 1.08; //Output voltage per rotation per second
    driveSlot0Configs.kP = 0.1; //Output voltage per rps of error
    driveSlot0Configs.kI = 0.0; //Output voltage for integral term
    driveSlot0Configs.kD = 0.0; //Output voltage for derivative term

    driveConfig.WithSlot0(driveSlot0Configs);
    m_request.WithSlot(0);

    //Turn Motor Motion Magic configs
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    
    configs::Slot0Configs turnSlot0Configs{};
    turnSlot0Configs.kP = 25;
    turnSlot0Configs.kD = 1;
    turnSlot0Configs.kV = 0.66;
    turnConfig.WithSlot0(turnSlot0Configs);
    m_turnRequest.WithSlot(0);

    turnConfig.Voltage.PeakForwardVoltage = 11_V;
    turnConfig.Voltage.PeakReverseVoltage = -11_V;

    turnConfig.Feedback = turnConfig.Feedback.WithFusedCANcoder(m_turningEncoder);
    turnConfig.Feedback.RotorToSensorRatio = swerveModule::kTurnGearRatio;

    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 6.5_tps;
    turnConfig.MotionMagic.MotionMagicAcceleration = 65_tr_per_s_sq;
    turnConfig.MotionMagic.MotionMagicExpo_kV = ctre::unit::volts_per_turn_per_second_t(0.12);
    turnConfig.MotionMagic.MotionMagicExpo_kA = ctre::unit::volts_per_turn_per_second_squared_t(0.1);

    m_driveMotor.GetConfigurator().Apply(driveConfig);
    m_turningMotor.GetConfigurator().Apply(turnConfig);

    m_driveMotor.SetNeutralMode(signals::NeutralModeValue::Brake);
    m_turningMotor.SetNeutralMode(signals::NeutralModeValue::Brake);

    if(m_driveMotor.GetDeviceID() == 1 || m_driveMotor.GetDeviceID() == 5)
    {
        m_driveMotor.SetInverted(true);
    }

    m_driveMotor.SetPosition(units::angle::turn_t{0});
}

void SwerveModule::ResetDriveEncoder()
{
    m_driveMotor.SetPosition(units::angle::turn_t{0});
}

void SwerveModule::Stop()
{
    m_driveMotor.StopMotor();
    m_turningMotor.StopMotor();
}

frc::SwerveModuleState SwerveModule::GetState() {
    return {
        units::meters_per_second_t{m_driveMotor.GetVelocity().GetValueAsDouble() * 2 * std::numbers::pi * swerveModule::kWheelRadius},
        units::radian_t{m_turningEncoder.GetAbsolutePosition().GetValueAsDouble() * (std::numbers::pi * 2)}
    };
}

// This returns the SwerveModulePosition object of the module, which includes drive distance and turn angle
frc::SwerveModulePosition SwerveModule::GetPosition() {
    
    return {
        units::meter_t{m_driveMotor.GetPosition().GetValueAsDouble() * 2 * std::numbers::pi * swerveModule::kWheelRadius},
        -units::radian_t{m_turningEncoder.GetAbsolutePosition().GetValueAsDouble() * (std::numbers::pi * 2)}
    };
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState)
{
    frc::Rotation2d currentAngle = frc::Rotation2d{units::radian_t{m_turningEncoder.GetAbsolutePosition().GetValue()}};  //Get the current module angle - Encoder returns in rotations, multiply by 2 pi to get radians
    frc::SwerveModuleState state = desiredState;

    state.Optimize(currentAngle) ;  //Optimize State, avoid spinning over 90 deg

    m_turningMotor.SetControl(m_turnRequest.WithPosition(units::turn_t{(double)state.angle.Radians() / (2 * std::numbers::pi)}));

    state.speed *= cos((double)(state.angle.Radians() - currentAngle.Radians()));

    units::turns_per_second_t driveTps = units::turns_per_second_t{(double)(state.speed / (std::numbers::pi * 2 * swerveModule::kWheelRadius))};
    m_driveMotor.SetControl(m_request.WithVelocity(driveTps));
}