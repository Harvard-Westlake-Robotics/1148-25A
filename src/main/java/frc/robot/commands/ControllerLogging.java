package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

public class ControllerLogging {

  public ControllerLogging() {}

  public static Command rightTriggerOn() {
    return Commands.run(
        () -> {
          Logger.recordOutput("ControllerInputs/RightTrigger", 1);
        });
  }

  public static Command rightTriggerOff() {
    return Commands.run(
        () -> {
          Logger.recordOutput("ControllerInputs/RightTrigger", 0);
        });
  }
}
