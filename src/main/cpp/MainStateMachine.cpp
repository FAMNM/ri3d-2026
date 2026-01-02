#include "MainStateMachine.h"

MainStateMachine::MainStateMachine()
{
	// Initialize Class Member Variables
   m_eState = RobotMain::eState::STATE_START;
   m_eDriveState = RobotMain::eDriveState::STATE_NORMAL; 
}

//-------------------------------------------------------------------

void MainStateMachine::Initialize(  )
{
   m_Drivetrain.Initialize();
}

//-------------------------------------------------------------------

void MainStateMachine::Execute()
{
   // *-------------*
   // * Start State *
   // *-------------*
   // There is currently nothing to be done to transition from the
   // Start State, so immediately transition to the Idle State.
   if ( m_eState == RobotMain::eState::STATE_START )
   {
      // printf( "Main - Enter Start State\n" );
      // Call the subsystem execute methods to allow them to advance
      // through the idle and start states.
      // printf( "Main - Advancing To Idle State\n" );
      m_eState = RobotMain::eState::STATE_IDLE;
   }
   // *------------*
   // * Idle State *
   // *------------*
   // Look for triggers for operations that are mutually exclusive (i.e.
   // cannot happen simultaneously because they use the same hardware).
   else if ( m_eState == RobotMain::eState::STATE_IDLE )
   {
      //Add controls here

   }

   // *--------------------*
   // * Driving Operations *
   // *--------------------*
   // Driver control to toggle field relative. Useful if Gyro gets offset or disconnected.
   if ( m_DriveController.GetStartButtonPressed() )
   {
      m_Drivetrain.ToggleFieldRelative();
   }
   
   m_Drivetrain.Execute(m_DriveController.GetLeftY(), m_DriveController.GetLeftX(), m_DriveController.GetRightX());
}
