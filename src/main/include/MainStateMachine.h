#ifndef MAIN_STATE_MACHINE_H_
#define MAIN_STATE_MACHINE_H_

#include <frc/XboxController.h>

#include "Drivetrain.h"                // Drivetrain state machine class
                                       //    definition

// *------------------------------------------------*
// * Top Level (Main) State Machine Enumerated Type *
// *------------------------------------------------*

namespace RobotMain
{
   enum eState
   {
      STATE_START = 0,
      STATE_IDLE = 1,
      STATE_ERROR = 99
   };

   // Different states for the drivetrain. Independent to avoid locking
   // driver out of drive controls.  Normal state is regular, states 1
   // & 2 are for reef, states 3 & 4 are for source stations.

   enum eDriveState  
   {
      STATE_NORMAL = 0,
   };
}

class MainStateMachine
{
   public:

      // Constructor/Destructor.

      MainStateMachine();
      ~MainStateMachine()
         { }

      // Class Methods.

      void Initialize();
      void UpdateStatus();
      void Execute();

   private:

      RobotMain::eState m_eState;      // Current main state
      RobotMain::eDriveState m_eDriveState;      // Current Drive state

      //X-Box Controllers
      frc::XboxController m_DriveController{0};
      frc::XboxController m_OperatorController{1};

      // State Machine Object Instances.
      Drivetrain m_Drivetrain;
};

#endif // MAIN_STATE_MACHINE_H_
