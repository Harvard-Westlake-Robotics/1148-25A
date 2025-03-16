package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ScoreCommand.ScoringLevel;
import frc.robot.subsystems.elevator.Elevator;

public class RaiseElevatorCommand extends Command {
  private double targetHeight;

  public RaiseElevatorCommand(ScoringLevel l4) {
    this.addRequirements(Elevator.getInstance());
    if (l4 == ScoringLevel.L1) {
      targetHeight = 15.5;
    } else if (l4 == ScoringLevel.L2) {
      targetHeight = 20.40;
    } else if (l4 == ScoringLevel.L3) {
      targetHeight = 30.08;
    } else if (l4 == ScoringLevel.L4) {
      targetHeight = 53.40;
    } else if (l4 == ScoringLevel.TOP_REMOVE) {
      targetHeight = 19.32;
    } else if (l4 == ScoringLevel.BOTTOM_REMOVE) {
      targetHeight = 7.80;
    } else {
      targetHeight = 0.0;
    }
    Elevator.getInstance().goToHeight(targetHeight);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Elevator.getInstance().goToHeight(targetHeight);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(targetHeight - Elevator.getInstance().getHeight()) < 5;
  }

  public void setHeight(ScoringLevel level) {
    if (level == ScoringLevel.L1) {
      targetHeight = 15.5;
    } else if (level == ScoringLevel.L2) {
      targetHeight = 20.40;
    } else if (level == ScoringLevel.L3) {
      targetHeight = 30.08;
    } else if (level == ScoringLevel.L4) {
      targetHeight = 53.40;
    } else if (level == ScoringLevel.TOP_REMOVE) {
      targetHeight = 19.32;
    } else if (level == ScoringLevel.BOTTOM_REMOVE) {
      targetHeight = 7.80;
    } else {
      targetHeight = 0.0;
    }
  }
}
