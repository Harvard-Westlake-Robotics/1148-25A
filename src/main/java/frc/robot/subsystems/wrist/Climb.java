package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final WristIOTalonFX io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private WristConstants constants;
  private String key;
  private static Climb instance;
  private DigitalInput limitSwitch = new DigitalInput(0);

  public static Climb getInstance() {
    if (instance == null) {
      instance = new Climb();
    }
    return instance;
  }

  public Climb() {
    this.constants = Constants.HangWrist;
    this.key = "Hang Wrist";
    io = new WristIOTalonFX(constants);
    io.zeroPosition(90 * 5);
    io.setAngle(90 * 5);
  }

  public WristConstants getConstants() {
    return constants;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);
  }

  public void goToAngle(double angle) {
    this.io.setAngle(angle);
  }

  public void runVoltage(double volts) {
    io.runCharacterization(volts);
  }

  public double getWristPosition() {
    return inputs.wristPositionMeters / constants.motorToWristRotations;
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }
}
