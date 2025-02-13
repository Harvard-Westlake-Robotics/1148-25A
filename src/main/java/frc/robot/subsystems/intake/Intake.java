package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeConstants constants;

  public Intake(IntakeConstants intakeConstants) {
    this.constants = intakeConstants;
    io = new IntakeIOTalonFX(intakeConstants);
  }

  public IntakeConstants getConstants() {
    return constants;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setVelocity(LinearVelocity velocity) {
    io.runVelocity(velocity);
  }
}
