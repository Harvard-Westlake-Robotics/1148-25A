package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase{
  private WristIOTalonFX io;
  private static Wrist instance = null;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public static Wrist getInstance() {
    if (instance == null) {
      instance = new Wrist();
    }
    return instance;
  }

  private Wrist() {
    io = new WristIOTalonFX(Constants.IntakeWrist);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
  }

  public void goToAngle(Angle angle) {
    // TODO: Implement
    io.setAngle(angle);
  }
}
