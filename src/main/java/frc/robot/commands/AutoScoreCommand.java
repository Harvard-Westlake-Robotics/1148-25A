package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ScoreCommand.ScoringLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.CoralIntake;

public class AutoScoreCommand extends Command {
  private ScoringLevel level;
  private double targetHeight = 0.0;
  private CoralIntakeCommand coralIntakeCommand = new CoralIntakeCommand(0);

  public AutoScoreCommand(ScoringLevel level) {
    this.addRequirements(CoralIntake.getInstance(), Elevator.getInstance());
    this.level = level;
  }

  @Override
  public void initialize() {
    coralIntakeCommand.schedule();
    if (level == ScoringLevel.L1) {
      targetHeight = 15.5;
    } else if (level == ScoringLevel.L2) {
      targetHeight = 20.85;
    } else if (level == ScoringLevel.L3) {
      targetHeight = 33.48;
    } else if (level == ScoringLevel.L4) {
      targetHeight = 53.40;
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
    Elevator.getInstance().goToHeight(targetHeight + Drive.getInstance().getElevatorHeight());
    if (Math.abs(Elevator.getInstance().getHeight() - targetHeight) < 0.2) {
      coralIntakeCommand.setEject(true);
      coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(100, MetersPerSecond));
    }
  }

  @Override
  public void end(boolean interrupted) {
    Elevator.getInstance().goToHeight(0);
    coralIntakeCommand.cancel();
  }

  @Override
  public boolean isFinished() {
    return !CoralIntake.getInstance().hasCoral();
  }
}
