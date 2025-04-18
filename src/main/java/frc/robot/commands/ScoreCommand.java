package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ScoreCommand extends Command {
  private double targetHeight;

  public enum ScoringLevel {
    L0,
    L1,
    L2,
    L3,
    L4,
    TOP_REMOVE,
    BOTTOM_REMOVE
  }

  public ScoreCommand(ScoringLevel level) {
    this.addRequirements(Elevator.getInstance());
    if (level == ScoringLevel.L1) {
      targetHeight = 15.5;
    } else if (level == ScoringLevel.L2) {
      targetHeight = 20.30;
    } else if (level == ScoringLevel.L3) {
      targetHeight = 30.08;
    } else if (level == ScoringLevel.L4) {
      targetHeight = 53.40;
    } else if (level == ScoringLevel.TOP_REMOVE) {
      targetHeight = 23.82;
    } else if (level == ScoringLevel.BOTTOM_REMOVE) {
      targetHeight = 13.10;
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
      targetHeight = 20.82;
    } else if (level == ScoringLevel.BOTTOM_REMOVE) {
      targetHeight = 9.80;
    } else {
      targetHeight = 0.0;
    }
  }
}
