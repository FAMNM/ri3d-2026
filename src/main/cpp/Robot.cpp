#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

// Provides initialization at robot power-on.

// Robot-wide initialization code goes here, which will be executed when
// the robot is first powered on.  It will be called exactly one time.

// Warning: the Driver Station "Robot Code" light and FMS "Robot Ready"
// indicators will be off until RobotInit() exits.  Code in RobotInit()
// that waits for enable will cause the robot to never indicate that
// the code is ready, causing the robot to be bypassed in a match.

void Robot::RobotInit()
{
   m_chooser.SetDefaultOption( AUTON_ROUTINE_01, AUTON_ROUTINE_01 );
   m_chooser.AddOption( AUTON_ROUTINE_01, AUTON_ROUTINE_01 );

   frc::SmartDashboard::PutData( "Auton Modes", &m_chooser );
}

//-------------------------------------------------------------------

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */

void Robot::RobotPeriodic()
{
   
}

//-------------------------------------------------------------------

// Method called once every time the Autonomous mode is entered from
// another mode.

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */

void Robot::AutonomousInit()
{
   frc::Shuffleboard::SelectTab("Auton");

   // printf( "\nEnter Robot::AutonomousInit\n" );

   m_autoSelected = m_chooser.GetSelected();
//-JJB    printf( "Autonomous Routine Selected: [%s]\n", m_autoSelected );
   // fmt::print( "\n>>> AutonomousInit -  Routine Selected: {}\n", m_autoSelected);

   // *-------------------------------------*
   // * Display Selected Autonomous Routine *
   // *-------------------------------------*

   frc::SmartDashboard::PutString(
      "Selected Auton Routine",             // Key name
      m_autoSelected );                     // Value

   if(m_autoSelected == AUTON_ROUTINE_01 ) { /*Initialize*/ }
}

//-------------------------------------------------------------------

void Robot::AutonomousPeriodic()
{
   // 01 - Move Out

   if ( m_autoSelected == AUTON_ROUTINE_01 )
   {
      //execute
   }
}

//-------------------------------------------------------------------

// Method called once every time the Teleop mode is entered from another
// mode,

void Robot::TeleopInit()
{
   // printf( "\nEnter Robot::TeleopInit\n" );

   m_MainStateMachine.Initialize();
   frc::Shuffleboard::SelectTab("Tele-Op");
}

//-------------------------------------------------------------------

void Robot::TeleopPeriodic()
{
   m_MainStateMachine.Execute();
}

//-------------------------------------------------------------------

// Method called once every time the Disabled mode is entered from another
// mode,

void Robot::DisabledInit()
{
   //  printf( "\nEnter Robot::DisabledInit\n" );
}

//-------------------------------------------------------------------

void Robot::DisabledPeriodic()
{
}

//-------------------------------------------------------------------

// Method called once every time the Test mode is entered from another
// mode,

void Robot::TestInit()
{
   
}

//-------------------------------------------------------------------

void Robot::TestPeriodic()
{
   
}

//-------------------------------------------------------------------

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
