package frc.robot.Camera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class LimeLightCam extends BaseCam {
  public String name = "";
  public static int LimeLightCount = 0;

  private NetworkTable _ntTable;

  private boolean useMegaTag2 = false;

  private static double X_MT1_VARIENCE_MAX = 0;
  private static double X_MT1_VARIENCE_MIN = 0;
  private static double X_MT2_VARIENCE_MAX = 0;
  private static double X_MT2_VARIENCE_MIN = 0;
  private static double Y_MT1_VARIENCE_MAX = 0;
  private static double Y_MT1_VARIENCE_MIN = 0;
  private static double Y_MT2_VARIENCE_MAX = 0;
  private static double Y_MT2_VARIENCE_MIN = 0;
  private static double T_MT1_VARIENCE_MAX = 0;
  private static double T_MT1_VARIENCE_MIN = 0;
  private static double T_MT2_VARIENCE_MAX = 0;
  private static double T_MT2_VARIENCE_MIN = 0;

  public boolean isUseMegaTag2() {
    return useMegaTag2;
  }

  public void setUseMegaTag2(boolean useMegaTag2) {
    this.useMegaTag2 = useMegaTag2;
  }

  // Resets the variance for everything
  public void resetVarience() {
    X_MT1_VARIENCE_MAX = Integer.MIN_VALUE;
    X_MT1_VARIENCE_MIN = Integer.MAX_VALUE;
    X_MT2_VARIENCE_MAX = Integer.MIN_VALUE;
    X_MT2_VARIENCE_MIN = Integer.MAX_VALUE;
    Y_MT1_VARIENCE_MAX = Integer.MIN_VALUE;
    Y_MT1_VARIENCE_MIN = Integer.MAX_VALUE;
    Y_MT2_VARIENCE_MAX = Integer.MIN_VALUE;
    Y_MT2_VARIENCE_MIN = Integer.MAX_VALUE;
    T_MT1_VARIENCE_MAX = Integer.MIN_VALUE;
    T_MT1_VARIENCE_MIN = Integer.MAX_VALUE;
    T_MT2_VARIENCE_MAX = Integer.MIN_VALUE;
    T_MT2_VARIENCE_MIN = Integer.MAX_VALUE;
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

    resetVarience();

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

    resetVarience();
  }

  public LimeLightCam(String name, boolean useMegaTag2) {
    this(name, new int[] {}, useMegaTag2);

    resetVarience();
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

  /* Returns the horizontal angle to the target in degrees */
  public double tX() {
    return -1 * _ntTable.getEntry("tx").getDouble(0);
  }

  /* Sets the robot's current yaw position when stationary */
  public void SetRobotOrientation(Rotation2d curYaw) {
    LimelightHelpers.SetRobotOrientation(name, curYaw.getDegrees(), 0, 0, 0, 0, 0);
  }

  /* Sets the robot's current yaw position and its rotational speed */
  public void SetRobotOrientation(Rotation2d curYaw, double curYawRate) {
    LimelightHelpers.SetRobotOrientation(name, curYaw.getDegrees(), curYawRate, 0, 0, 0, 0);
  }

  /*
   * Obtains an estimated position based off tag readings.
   * When disabled, we run MegaTag1 because there shouldn't be any ambiguity
   * variables besides distance.
   * When enabled, MegaTag2 automatically filters out invalid or ambiguous
   * readings
   */
  public Optional<AprilTagResult> getEstimate() {
    if (runNeuralNetwork) {
      return Optional.empty();
    }
    LimelightHelpers.PoseEstimate latestEstimate;

    // Selects which MegaTag to use
    // if (DriverStation.isDisabled()) {
    latestEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    // } else {
    // latestEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    // }

    if (latestEstimate == null) return Optional.empty();

    if (latestEstimate.tagCount == 0) return Optional.empty();

    // Logs the variance, stdDev, ambiguity, and tag distance of a given limelight;
    // MT 1 and 2 have
    // seperated values
    if (DriverStation.isDisabled()) {
      if (latestEstimate.pose.getX() > X_MT1_VARIENCE_MAX) {
        X_MT1_VARIENCE_MAX = latestEstimate.pose.getX();
      }
      if (latestEstimate.pose.getX() < X_MT1_VARIENCE_MIN) {
        X_MT1_VARIENCE_MIN = latestEstimate.pose.getX();
      }
      if (latestEstimate.pose.getY() > Y_MT1_VARIENCE_MAX) {
        Y_MT1_VARIENCE_MAX = latestEstimate.pose.getY();
      }
      if (latestEstimate.pose.getY() < Y_MT1_VARIENCE_MIN) {
        Y_MT1_VARIENCE_MIN = latestEstimate.pose.getY();
      }
      if (latestEstimate.pose.getRotation().getRadians() > T_MT1_VARIENCE_MAX) {
        T_MT1_VARIENCE_MAX = latestEstimate.pose.getRotation().getRadians();
      }
      if (latestEstimate.pose.getRotation().getRadians() < T_MT1_VARIENCE_MIN) {
        T_MT1_VARIENCE_MIN = latestEstimate.pose.getRotation().getRadians();
      }
      Logger.recordOutput(
          "RealOutputs/" + name + "/MT1/XVarience", X_MT1_VARIENCE_MAX - X_MT1_VARIENCE_MIN);
      Logger.recordOutput(
          "RealOutputs/" + name + "/MT1/XStdDev",
          Math.sqrt(X_MT1_VARIENCE_MAX - X_MT1_VARIENCE_MIN));
      Logger.recordOutput(
          "RealOutputs/" + name + "/MT1/YVarience", Y_MT1_VARIENCE_MAX - Y_MT1_VARIENCE_MIN);
      Logger.recordOutput(
          "RealOutputs/" + name + "/MT1/YStdDev",
          Math.sqrt(Y_MT1_VARIENCE_MAX - Y_MT1_VARIENCE_MIN));
      Logger.recordOutput(
          "RealOutputs/" + name + "/MT1/AngleVarience", T_MT1_VARIENCE_MAX - T_MT1_VARIENCE_MIN);
      Logger.recordOutput(
          "RealOutputs/" + name + "/MT1/AngleStdDev",
          Math.sqrt(T_MT1_VARIENCE_MAX - T_MT1_VARIENCE_MIN));
      Logger.recordOutput(
          "RealOutputs/" + name + "/MT1/ambiguity", latestEstimate.rawFiducials[0].ambiguity);
      Logger.recordOutput("RealOutputs/" + name + "/MT1/tagDistance", latestEstimate.avgTagDist);
      // } else {
      // if (latestEstimate.pose.getX() > X_MT2_VARIENCE_MAX) {
      // X_MT2_VARIENCE_MAX = latestEstimate.pose.getX();
      // }
      // if (latestEstimate.pose.getX() < X_MT2_VARIENCE_MIN) {
      // X_MT2_VARIENCE_MIN = latestEstimate.pose.getX();
      // }
      // if (latestEstimate.pose.getX() > Y_MT2_VARIENCE_MAX) {
      // Y_MT2_VARIENCE_MAX = latestEstimate.pose.getY();
      // }
      // if (latestEstimate.pose.getY() < Y_MT2_VARIENCE_MIN) {
      // Y_MT2_VARIENCE_MIN = latestEstimate.pose.getY();
      // }
      // if (latestEstimate.pose.getRotation().getRadians() > T_MT2_VARIENCE_MAX) {
      // T_MT2_VARIENCE_MAX = latestEstimate.pose.getRotation().getRadians();
      // }
      // if (latestEstimate.pose.getRotation().getRadians() < T_MT2_VARIENCE_MIN) {
      // T_MT2_VARIENCE_MIN = latestEstimate.pose.getRotation().getRadians();
      // }
      // Logger.recordOutput(
      // "RealOutputs/" + name + "/MT2/XVarience", X_MT2_VARIENCE_MAX -
      // X_MT2_VARIENCE_MIN);
      // Logger.recordOutput(
      // "RealOutputs/" + name + "/MT2/XStdDev",
      // Math.sqrt(X_MT2_VARIENCE_MAX - X_MT2_VARIENCE_MIN));
      // Logger.recordOutput(
      // "RealOutputs/" + name + "/MT2/YVarience", Y_MT2_VARIENCE_MAX -
      // Y_MT2_VARIENCE_MIN);
      // Logger.recordOutput(
      // "RealOutputs/" + name + "/MT2/YStdDev",
      // Math.sqrt(Y_MT2_VARIENCE_MAX - Y_MT2_VARIENCE_MIN));
      // Logger.recordOutput(
      // "RealOutputs/" + name + "/MT2/AngleVarience", T_MT2_VARIENCE_MAX -
      // T_MT2_VARIENCE_MIN);
      // Logger.recordOutput(
      // "RealOutputs/" + name + "/MT2/AngleStdDev",
      // Math.sqrt(T_MT2_VARIENCE_MAX - T_MT2_VARIENCE_MIN));
      // Logger.recordOutput(
      // "RealOutputs/" + name + "/MT2/ambiguity",
      // latestEstimate.rawFiducials[0].ambiguity);
      // Logger.recordOutput("RealOutputs/" + name + "/MT2/tagDistance",
      // latestEstimate.avgTagDist);
    }
    return Optional.of(
        new AprilTagResult(
            latestEstimate.pose,
            latestEstimate.timestampSeconds,
            latestEstimate.avgTagDist,
            latestEstimate.tagCount,
            latestEstimate.rawFiducials[0].ambiguity));
  }

  /* Returns the latest tag detections from the limelight */
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
