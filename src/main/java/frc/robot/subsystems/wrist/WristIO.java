package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean wristMotorConnected = false;
    public double wristPositionMeters = 0.0;
    public double wristVelocityMPS = 0.0;
    public double wristAppliedVolts = 0.0;
    public double wristCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  public default void setAngle(double angle) {}

  public default void runCharacterization(double volts) {}

  public default void zeroPosition(double rotations) {}
}
