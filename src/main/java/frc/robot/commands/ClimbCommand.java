package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Climb;

public class ClimbCommand extends Command {
  private Climb climb;

  public ClimbCommand() {
    this.addRequirements(Climb.getInstance());
    this.climb = Climb.getInstance();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  public void deploy() {
    Command deployCommand =
        new Command() {
          @Override
          public void initialize() {
            Climb.getInstance().goToAngle(23);
          }

          @Override
          public void end(boolean interrupted) {
            Climb.getInstance().goToAngle(0);
          }
        }.withTimeout(2);
    deployCommand.schedule();
  }

  public void climb() {
    climb.goToAngle(63);
  }
}
