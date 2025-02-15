package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private WristIOTalonFX io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private WristConstants constants;
  private String key;

  public Wrist(WristConstants constants, String key) {
    this.constants = constants;
    this.key = key;
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
}
