package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
  public static class TimestampedPose {
    public Pose2d pose;
    public double timestamp;

    public TimestampedPose(Pose2d pose, double timestamp) {
      this.pose = pose;
      this.timestamp = timestamp;
    }
  }

  @AutoLog
  public static class CameraIOInputs {
    public int tagCount = 0;
    public double ambiguity = 0.0;
    public Distance distToCamera = Meters.of(0.0);
    public Pose2d estimatedPose;
  }

  public default void updateInputs(CameraIOInputs inputs) {}

  public default TimestampedPose getTimestampedPose() {
    return null;
  }
}
