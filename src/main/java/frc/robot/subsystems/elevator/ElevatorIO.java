package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean motor1Connected = false;
    public boolean motor2Connected = false;
    public double elevatorPositionMeters = 0.0;
    public double elevatorVelocityMPS = 0.0;
    public double elevatorAppliedVolts = 0.0;
    public double elevatorCurrentAmps = 0.0;
    public boolean inputValue = false;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void goToHeight(double heightMeters) {}

  public default void runVoltage(double volts) {}

  public default double getTarget() {
    return 0.0;
  }

  public default boolean getInput() {
    return false;
  }
}
