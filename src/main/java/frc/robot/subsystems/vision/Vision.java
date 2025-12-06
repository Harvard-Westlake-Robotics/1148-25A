package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.CameraIO.TimestampedPose;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private static Vision instance;

  private CameraIOLimelight[] cameras;
  private CameraIOInputsAutoLogged[] inputs;

  private String key = "Vision";

  public static Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  public Vision() {
    cameras = new CameraIOLimelight[Constants.Vision.limelightNames.length];
    inputs = new CameraIOInputsAutoLogged[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      cameras[i] = new CameraIOLimelight(Constants.Vision.limelightNames[i]);
      inputs[i] = new CameraIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < inputs.length; i++) {
      cameras[i].updateInputs(inputs[i]);
      Logger.processInputs(key + "/" + cameras[i].getName(), inputs[i]);
    }
  }

  public void setUseMegaTag2(boolean useMegaTag2) {
    for (CameraIOLimelight camera : cameras) {
      camera.setUseMegaTag2(useMegaTag2);
    }
  }

  public void setRobotOrientation(Rotation2d yaw) {
    for (CameraIOLimelight camera : cameras) {
      camera.setRobotOrientation(yaw);
    }
  }

  @AutoLogOutput
  public TimestampedPose[] getTimestampedPoses() {
    TimestampedPose[] poses = new TimestampedPose[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      poses[i] = cameras[i].getTimestampedPose();
    }
    return poses;
  }
}
