package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.NetworkCommunicator;
import frc.robot.subsystems.intake.CoralIntake;

public class TeleopCommand extends Command {
  public TeleopCommand() {}

  private Command reefCommand =
      AutoBuilder.pathfindThenFollowPath(
              NetworkCommunicator.getInstance().getSelectedReefPath(), Drive.PP_CONSTRAINTS)
          .andThen(new AutoScoreCommand(NetworkCommunicator.getInstance().getSelectedHeight()));
  private Command sourceCommand =
      AutoBuilder.pathfindThenFollowPath(
              NetworkCommunicator.getInstance().getSelectedSourcePath(), Drive.PP_CONSTRAINTS)
          .andThen(new CoralIntakeCommand(20));

  @Override
  public void initialize() {
    if (CoralIntake.getInstance().hasCoral()) {
      reefCommand.schedule();
    } else {
      sourceCommand.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    sourceCommand.cancel();
    reefCommand.cancel();
  }

  public void updateCommands() {
    reefCommand =
        AutoBuilder.pathfindThenFollowPath(
                NetworkCommunicator.getInstance().getSelectedReefPath(), Drive.PP_CONSTRAINTS)
            .andThen(new AutoScoreCommand(NetworkCommunicator.getInstance().getSelectedHeight()));
    sourceCommand =
        AutoBuilder.pathfindThenFollowPath(
                NetworkCommunicator.getInstance().getSelectedSourcePath(), Drive.PP_CONSTRAINTS)
            .andThen(new CoralIntakeCommand(20));
  }
}
