package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeMotorConnected = false;
    public double intakePositionMeters = 0.0;
    public double intakeVelocityMPS = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {
  }

  public default void runVelocity(AngularVelocity velocity) {
  }

  public default void runVelocity(LinearVelocity velocity) {
  }

  public default void runOpenLoop(AngularVelocity velocity) {
  }

  public default void runOpenLoop(LinearVelocity velocity) {
  }
}
