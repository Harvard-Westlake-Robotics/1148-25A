package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  private IntakeIOTalonFX io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeConstants constants;
  private String key;
  private static CoralIntake instance;
  private boolean hasCoral;
  SysIdRoutine sysId;

  public static CoralIntake getInstance() {
    if (instance == null) {
      instance = new CoralIntake();
    }
    return instance;
  }

  public CoralIntake() {
    this.constants = Constants.CoralIntake;
    this.key = "Coral Intake";
    io = new IntakeIOTalonFX(constants);
    sysId = new SysIdRoutine(
        new Config(null, null, null, (state) -> Logger.recordOutput("CoralIntake/SysIdState", state.toString())),
        new Mechanism(
            (voltage) -> runVoltage(voltage.in(Volts)), null, this));
  }

  public IntakeConstants getConstants() {
    return constants;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);
    if (!getSensor2()) {
      hasCoral = true;
    } else {
      hasCoral = false;
    }
  }

  public void setVelocity(LinearVelocity velocity) {
    io.runVelocity(velocity);
  }

  public void push(double rotations) {
    io.push(rotations);
  }

  public void runVoltage(double volts) {
    io.runCharacterization(volts);
  }

  public Boolean getSensor1() {
    return io.getSensor1();
  }

  public Boolean getSensor2() {
    return io.getSensor2();
  }

  public Boolean getSensor3() {
    return io.getSensor3();
  }

  public Boolean hasCoral() {
    return hasCoral;
  }

  public void runOver(LinearVelocity v) {
    io.runVelocityOverride(v);
  }

  public void setOverride(boolean over) {
    io.setOverride(over);
  }

  public boolean isOverride() {
    return io.isOverride();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runVoltage(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }
}
