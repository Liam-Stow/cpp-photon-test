#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot() {
  _visionSim.AddAprilTags(_tagLayout);
  _visionSim.AddCamera(&_cameraSim, _botToCam);
  frc::SmartDashboard::PutData("field", &_fieldDisplay);
}

std::vector<photon::PhotonPipelineResult> Robot::OrderResultsByTimestamp(
    std::vector<photon::PhotonPipelineResult> results) {
  std::sort(
      results.begin(), results.end(),
      [](photon::PhotonPipelineResult &a, photon::PhotonPipelineResult &b) {
        return a.GetTimestamp() > b.GetTimestamp();
      });
  return results;
}

void Robot::RobotPeriodic() {
  // Photonvision updates
  auto results = _camera.GetAllUnreadResults();
  auto resultsOrderedByTimestamp = OrderResultsByTimestamp(results);
  for (auto result : resultsOrderedByTimestamp) {
    auto visionPose = _robotPoseEstimater.Update(result);
    if (visionPose.has_value()) {
      _fieldDisplay.GetObject("vision estimate")
          ->SetPose(visionPose.value().estimatedPose.ToPose2d());
    }
  }

  // Pretend a robot is driving across the field
  _trueRobotPose = _trueRobotPose + frc::Transform2d{0.005_m, 0_m, 0_deg};
  _fieldDisplay.SetRobotPose(_trueRobotPose);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  _trueRobotPose = frc::Pose2d{0_m, 5_m, 0_deg};
}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() { _visionSim.Update(_trueRobotPose); }

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
