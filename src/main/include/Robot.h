#pragma once

#include <frc/TimedRobot.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/simulation/VisionSystemSim.h>

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

  std::vector<photon::PhotonPipelineResult> OrderResultsByTimestamp(
      std::vector<photon::PhotonPipelineResult> results);

  frc::Pose2d _trueRobotPose{0_m, 5_m, 0_deg};
  frc::Field2d _fieldDisplay;

  std::string _cameraName = "camera";
  photon::PhotonCamera _camera{_cameraName};

  frc::Transform3d _botToCam;
  frc::AprilTagFieldLayout _tagLayout =
      frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);

  photon::PhotonPoseEstimator _robotPoseEstimater{
      _tagLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
      _botToCam};

  photon::PhotonCameraSim _cameraSim{&_camera};
  photon::VisionSystemSim _visionSim{_cameraName};
};
