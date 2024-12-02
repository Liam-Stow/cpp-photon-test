#pragma once

#include <frc/TimedRobot.h>

// Culprit:
#include <photon/simulation/PhotonCameraSim.h>

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;
};
