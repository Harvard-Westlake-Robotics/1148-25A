package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean elevator1Connected = false;
    public double elevator1PositionMeters = 0.0;
    public double elevator1VelocityMPS = 0.0;
    public double elevator1AppliedVolts = 0.0;
    public double elevator1CurrentAmps = 0.0;

    public boolean elevator2Connected = false;
    public double elevator2PositionMeters = 0.0;
    public double elevator2VelocityMPS = 0.0;
    public double elevator2AppliedVolts = 0.0;
    public double elevator2CurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run the drive motor at the specified velocity. */
  public default void setHeightClosedLoop(double velocityRadPerSec) {}

  public default void setHeightMetersAdjusted(double meters) {}

  public default void zeroMotors() {}

  public default void setIsOverriding(boolean isOverriding) {}

  public default void setHeightClosedLoopOverride(double velocity) {}
}
