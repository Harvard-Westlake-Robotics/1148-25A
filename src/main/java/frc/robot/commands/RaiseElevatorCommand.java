package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ScoreCommand.ScoringLevel;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.CoralIntake;

public class RaiseElevatorCommand extends Command {
  private double targetHeight;

  public RaiseElevatorCommand(ScoringLevel level) {
    this.addRequirements(Elevator.getInstance());
    // If coral not fully in robot, prevent robot from sending elevator anywhere
    if (CoralIntake.getInstance().getSensor3()) {
      if (level == ScoringLevel.L1) {
        targetHeight = 15.5;
      } else if (level == ScoringLevel.L2) {
        targetHeight = 20.40;
      } else if (level == ScoringLevel.L3) {
        targetHeight = 31.30;
      } else if (level == ScoringLevel.L4) {
        targetHeight = 52.50;
      } else if (level == ScoringLevel.TOP_REMOVE) {
        targetHeight = 19.32;
      } else if (level == ScoringLevel.BOTTOM_REMOVE) {
        targetHeight = 7.80;
      } else {
        targetHeight = 0.0;
      }
    }
    Elevator.getInstance().goToHeight(targetHeight);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (targetHeight == 0.0 && !Elevator.getInstance().getInput()) {
      targetHeight = Elevator.getInstance().getHeight();
    }
    Elevator.getInstance().goToHeight(targetHeight);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(targetHeight - Elevator.getInstance().getHeight()) < 2;
  }

  public void setHeight(ScoringLevel level) {
    // If coral not fully in robot, prevent robot from sending elevator anywhere
    if (CoralIntake.getInstance().getSensor3()) {
      if (level == ScoringLevel.L1) {
        targetHeight = 15.5;
      } else if (level == ScoringLevel.L2) {
        targetHeight = 20.40;
      } else if (level == ScoringLevel.L3) {
        targetHeight = 31.30;
      } else if (level == ScoringLevel.L4) {
        targetHeight = 52.50;
      } else if (level == ScoringLevel.TOP_REMOVE) {
        targetHeight = 19.32;
      } else if (level == ScoringLevel.BOTTOM_REMOVE) {
        targetHeight = 7.80;
      } else {
        targetHeight = 0.0;
      }
    }
  }
}
