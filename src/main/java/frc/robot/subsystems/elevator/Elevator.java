package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private static Elevator instance = null;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private DigitalInput dio = new DigitalInput(5);

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  private Elevator() {
    io = new ElevatorIOTalonFX();
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
}
