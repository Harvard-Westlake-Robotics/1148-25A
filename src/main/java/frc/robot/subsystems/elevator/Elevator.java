package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private static Elevator instance = null;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private DigitalInput dio = new DigitalInput(5);
  SysIdRoutine sysId;

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  private Elevator() {
    io = new ElevatorIOTalonFX();
    sysId = new SysIdRoutine(
        new Config(null, null, null, (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
        new Mechanism(
            (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    if (!dio.get()) {
      io.zeroMotors();
    }
  }

  public void goToHeight(double height) {
    io.setHeightClosedLoop(height);
  }

  public void goToHeightMeters(double height) {
    io.setHeightMetersAdjusted(height);
  }

  public double getHeight() {
    return inputs.elevator1PositionMeters;
  }

  public void goToHeightOverride(double height) {
    io.setHeightClosedLoopOverride(getHeight());
  }

  public void setOverride(boolean over) {
    io.setIsOverriding(over);
  }

  public void runCharacterization(double voltage) {
    io.runCharacterization(voltage);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }
}
