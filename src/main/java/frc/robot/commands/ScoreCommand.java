package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.CoralIntake;

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
    addRequirements(Elevator.getInstance());
    if (level == ScoringLevel.L1) {
      targetHeight = 15.5;
    } else if (level == ScoringLevel.L2) {
      targetHeight = 20.40;
    } else if (level == ScoringLevel.L3) {
      targetHeight = 32.08;
    } else if (level == ScoringLevel.L4) {
      targetHeight = 53.40;
    } else if (level == ScoringLevel.TOP_REMOVE) {
      targetHeight = 19.32;
    } else if (level == ScoringLevel.BOTTOM_REMOVE) {
      targetHeight = 7.80;
    } else {
      targetHeight = 0.0;
    }
    Elevator.getInstance().goToHeight(targetHeight);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return !CoralIntake.getInstance().getSensor1();
  }
}
