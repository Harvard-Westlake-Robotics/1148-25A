package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import org.littletonrobotics.junction.Logger;

public class AlgaeWrist extends SubsystemBase {
  private WristIOTalonFX io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private WristConstants constants;
  private String key;
  private static AlgaeWrist instance;

  public static AlgaeWrist getInstance() {
    if (instance == null) {
      instance = new AlgaeWrist();
    }
    return instance;
  }

  public AlgaeWrist() {
    this.constants = Constants.IntakeWrist;
    this.key = "Intake Wrist";
    io = new WristIOTalonFX(constants);
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
}
