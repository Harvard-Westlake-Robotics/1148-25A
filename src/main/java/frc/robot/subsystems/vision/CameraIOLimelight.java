package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class CameraIOLimelight implements CameraIO {
  private final String name;

  public String getName() {
    return name;
  }

  private boolean useMegaTag2 = false;

  public void setUseMegaTag2(boolean useMegaTag2) {
    this.useMegaTag2 = useMegaTag2;
  }

  private PoseEstimate currentPose;

  public CameraIOLimelight(String name) {
    this.name = name;
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    if (useMegaTag2) {
      currentPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    } else {
      currentPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    }
    inputs.tagCount = currentPose.tagCount;
    inputs.ambiguity = currentPose.rawFiducials[0].ambiguity;
    inputs.distToCamera = Meters.of(currentPose.rawFiducials[0].distToCamera);
    inputs.estimatedPose = currentPose.pose;
  }

  // Shutter Speed
  public void setThrottle(int throttle) {
    LimelightHelpers.SetThrottle(name, throttle);
  }

  public void setRobotOrientation(Rotation2d yaw) {
    LimelightHelpers.SetRobotOrientation(name, yaw.getDegrees(), 0, 0, 0, 0, 0);
  }

  @Override
  public TimestampedPose getTimestampedPose() {
    if (!shouldAcceptPose(currentPose)) {
      return null;
    }
    return new TimestampedPose(currentPose.pose, currentPose.timestampSeconds);
  }

  private boolean shouldAcceptPose(PoseEstimate pose) {
    if (pose.tagCount == 0) {
      return false;
    }
    if (pose.rawFiducials[0].ambiguity > Constants.Vision.maxAmbiguity) {
      return false;
    }
    if (pose.rawFiducials[0].distToCamera > Constants.Vision.maxDistToCamera.in(Meters)) {
      return false;
    }
    return true;
  }
}
