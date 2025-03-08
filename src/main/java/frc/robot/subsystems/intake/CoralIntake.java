package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeConstants constants;
  private String key;
  private static CoralIntake instance;
  private boolean hasCoral;

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

  public Boolean getSensor3(){
    return io.getSensor3();
  }

  public Boolean hasCoral() {
    return hasCoral;
  }
}
