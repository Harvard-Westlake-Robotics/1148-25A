package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private static Elevator instance;

  private ElevatorIOTalonFX io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

//   private SysIdRoutine sysid;

  private String key = "Elevator";

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  public Elevator() {
    io = new ElevatorIOTalonFX();

    sysid =
        new SysIdRoutine(
            new Config(null, null, null),
            new Mechanism(
                (volts) -> {
                  io.runVoltage(volts.in(Volts));
                },
                null,
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);
  }

  public void goToHeight(double heightMeters) {
    io.goToHeight(heightMeters);
  }

  public double getHeight() {
    return inputs.elevatorPositionMeters;
  }

  @AutoLogOutput
  public double getTarget() {
    return io.getTarget();
  }

  public boolean getInput() {
    return inputs.inputValue;
  }
}
