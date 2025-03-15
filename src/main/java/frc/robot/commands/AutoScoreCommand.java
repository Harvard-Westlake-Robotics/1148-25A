package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.ScoreCommand.ScoringLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.CoralIntake;

public class AutoScoreCommand extends Command {
  private ScoringLevel level;
  private double targetHeight = 0.0;
  // private CoralIntakeCommand coralIntakeCommand = new CoralIntakeCommand(0);

  public AutoScoreCommand(ScoringLevel level) {
    this.addRequirements(CoralIntake.getInstance(), Elevator.getInstance());
    this.level = level;
  }

  @Override
  public void initialize() {
    // coralIntakeCommand.schedule();
    if (level == ScoringLevel.L1) {
      targetHeight = 0.454; // 17.5;
    } else if (level == ScoringLevel.L2) {
      targetHeight = 0.79; // 23.85;
    } else if (level == ScoringLevel.L3) {
      targetHeight = 1.193; // 34.48;
    } else if (level == ScoringLevel.L4) {
      targetHeight = 1.83; // 52.40;
    } else if (level == ScoringLevel.TOP_REMOVE) {
      targetHeight = 19.12;
    } else if (level == ScoringLevel.BOTTOM_REMOVE) {
      targetHeight = 7.60;
    } else {
      targetHeight = 0.0;
    }
    Elevator.getInstance().goToHeight(targetHeight);
  }

  @Override
  public void execute() {
    if (level == ScoringLevel.L1) {
      targetHeight = 0.454 + Drive.getInstance().getElevatorHeight(); // 17.5;
    } else if (level == ScoringLevel.L2) {
      targetHeight = 0.79 + Drive.getInstance().getElevatorHeight(); // 23.85;
    } else if (level == ScoringLevel.L3) {
      targetHeight = 1.193 + Drive.getInstance().getElevatorHeight(); // 34.48;
    } else if (level == ScoringLevel.L4) {
      targetHeight = 1.83 + Drive.getInstance().getElevatorHeight(); // 52.40;
    } else if (level == ScoringLevel.TOP_REMOVE) {
      targetHeight = 19.12;
    } else if (level == ScoringLevel.BOTTOM_REMOVE) {
      targetHeight = 7.60;
    } else {
      targetHeight = 0.0;
    }
    Elevator.getInstance().goToHeightMeters(targetHeight);
    if (Math.abs(
            Elevator.getInstance().getHeight()
                - (((targetHeight - Constants.Elevator.elevatorGroundOffsetMeters)
                    * Constants.Elevator.rotationsToMetersRatio)))
        < 1.5) {
      CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(50, MetersPerSecond));
    }
  }

  @Override
  public void end(boolean interrupted) {
    Elevator.getInstance().goToHeight(0);
    CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
  }

  @Override
  public boolean isFinished() {
    return !CoralIntake.getInstance().hasCoral();
  }
}
