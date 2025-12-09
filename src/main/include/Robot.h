//---------------------------------------------------------------------------
//
// Robot.h - Robot Class Definition.
//
//---------------------------------------------------------------------------
//
// Date       Name      Changes
// ---------  --------  -------------------------------------------
// 28-Feb-25  JJB       Added file header with revision log.
//
//---------------------------------------------------------------------------
//
// Description:
//    This class provides the following functions which are called by the
// main loop at the appropriate times.
//
// Documentation:
// https://first.wpi.edu/wpilib/allwpilib/docs/release/cpp/classfrc_1_1_iterative_robot_base.html
// https://docs.wsr.studica.com/en/latest/docs/Software/programming/base-project-outline/robot.html
//
//---------------------------------------------------------------------------

#pragma once

#include <string>

#include <numbers>

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/XboxController.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <frc/kinematics/SwerveModuleState.h>

#include "MainStateMachine.h"          // Main State Machine class definition

using namespace ctre::phoenix6;

class Robot : public frc::TimedRobot
{
   public:

      void RobotInit() override;
      void RobotPeriodic() override;
      void AutonomousInit() override;
      void AutonomousPeriodic() override;
      void TeleopInit() override;
      void TeleopPeriodic() override;
      void DisabledInit() override;
      void DisabledPeriodic() override;
      void TestInit() override;
      void TestPeriodic() override;

   private:

      MainStateMachine m_MainStateMachine;   // Main Robot State Machine

      // Autonomous Mode.

      frc::SendableChooser<std::string> m_chooser;
      const std::string AUTON_ROUTINE_01 = "01 - AUTON";
      // Add more auton names
      std::string m_autoSelected;
};
