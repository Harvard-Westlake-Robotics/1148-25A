package frc.robot.Camera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Camera.BaseCam.NeuralDetectorResult;
import java.util.Optional;

/* Maybe in future remove limelightHelpers Dependency,
really just copy pasting but I'm lazy */

public class LimeLightCam extends BaseCam {
  public String name = "";
  public static int LimeLightCount = 0;

  private NetworkTable _ntTable;

  private boolean useMegaTag2 = false;

  public boolean isUseMegaTag2() {
    return useMegaTag2;
  }

  public void setUseMegaTag2(boolean useMegaTag2) {
    this.useMegaTag2 = useMegaTag2;
  }

  private boolean runNeuralNetwork = false;

  public LimeLightCam(String name, int[] TagsToCheck, boolean useMegaTag2) {
    this.name = name;

    if (TagsToCheck.length > 0) {
      LimelightHelpers.SetFiducialIDFiltersOverride(name, TagsToCheck);
    }

    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port + 10 * LimeLightCount, String.format("%s.local", this.name), port);
    }

    _ntTable = NetworkTableInstance.getDefault().getTable(name);

    _ntTable
        .getEntry("ledMode")
        .setNumber(1); // Example of how to use -> This turns off LEDS on the front

    this.useMegaTag2 = useMegaTag2;

    LimeLightCount++;
    setIMUMode(1);
  }

  public LimeLightCam(String name) {
    this(name, new int[] {}, false);
  }

  public LimeLightCam(String name, boolean useMegaTag2) {
    this(name, new int[] {}, useMegaTag2);
  }

  public void setNeuralNetwork(boolean running) {
    this.runNeuralNetwork = running;
    if (running) LimelightHelpers.setPipelineIndex(name, 1);
    else LimelightHelpers.setPipelineIndex(name, 0);
  }

  public void setIMUMode(int mode) {
    LimelightHelpers.SetIMUMode(name, mode);
  }

  public int targetCount() {
    double[] t2d = _ntTable.getEntry("t2d").getDoubleArray(new double[0]);
    if (t2d.length == 17) {
      return (int) t2d[1];
    }
    return 0;
  }

  public boolean hasTargets() {
    return targetCount() > 0;
  }

  public double tX() {
    return -1 * _ntTable.getEntry("tx").getDouble(0);
  }

  /* Used with MegaTag2, don't know how that thing works tho */
  public void SetRobotOrientation(Rotation2d curYaw) {
    LimelightHelpers.SetRobotOrientation(name, curYaw.getDegrees(), 0, 0, 0, 0, 0);
  }

  public void SetRobotOrientation(Rotation2d curYaw, double curYawRate) {
    LimelightHelpers.SetRobotOrientation(name, curYaw.getDegrees(), curYawRate, 0, 0, 0, 0);
  }

  public Optional<AprilTagResult> getEstimate() {
    if (runNeuralNetwork) {
      return Optional.empty();
    }
    LimelightHelpers.PoseEstimate latestEstimate;
    if (DriverStation.isDisabled()) {
      latestEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    } else {
      latestEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }

    if (latestEstimate == null) return Optional.empty();

    if (latestEstimate.tagCount == 0) return Optional.empty();

    return Optional.of(
        new AprilTagResult(
            latestEstimate.pose,
            latestEstimate.timestampSeconds,
            latestEstimate.avgTagDist,
            latestEstimate.tagCount,
            latestEstimate
                .rawFiducials[0]
                .ambiguity)); // Probably not the best but good enough for now
  }

  public Optional<AprilTagResult> getEstimateMT2() {
    if (runNeuralNetwork) {
      return Optional.empty();
    }
    LimelightHelpers.PoseEstimate latestEstimate;
    latestEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    if (latestEstimate == null) return Optional.empty();

    if (latestEstimate.tagCount == 0) return Optional.empty();

    return Optional.of(
        new AprilTagResult(
            latestEstimate.pose,
            latestEstimate.timestampSeconds,
            latestEstimate.avgTagDist,
            latestEstimate.tagCount,
            latestEstimate
                .rawFiducials[0]
                .ambiguity)); // Probably not the best but good enough for now
  }

  public Optional<NeuralDetectorResult[]> getDetections() {
    if (!runNeuralNetwork) {
      return Optional.empty();
    }

    LimelightHelpers.LimelightTarget_Detector[] latestResult;
    latestResult = LimelightHelpers.getLatestResults(name).targets_Detector;

    if (latestResult == null) return Optional.empty();
    if (latestResult.length == 0) return Optional.empty();

    NeuralDetectorResult[] results = new NeuralDetectorResult[latestResult.length];
    for (int i = 0; i < results.length; i++) {
      results[i] =
          new NeuralDetectorResult(
              latestResult[i].className,
              latestResult[i].classID,
              latestResult[i].confidence,
              latestResult[i].ta,
              latestResult[i].tx,
              latestResult[i].ty);
    }

    return Optional.of(results);
  }
}
