package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase{
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

  public void goToAngle(Angle angle) {
    io.setAngle(angle);
  }
}
