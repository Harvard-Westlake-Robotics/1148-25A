package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Climb;

public class ClimbCommand extends Command {
  private Climb climb;
  private boolean deploy;
  private boolean climbDown;

  public ClimbCommand() {
    this.addRequirements(Climb.getInstance());
    this.climb = Climb.getInstance();
    deploy = false;
    climbDown = false;
  }

  @Override
  public void initialize() {
    climb.goToAngle(0);
  }

  @Override
  public void execute() {
    if (deploy) {
      climb.goToAngle(20);
      if (climb.getWristPosition() > 15) {
        deploy = false;
      }
    } else if (climbDown) {
      climb.goToAngle(177.1);
    } else {
      climb.goToAngle(0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  public void deploy() {
    deploy = true;
  }

  public void climb() {
    climbDown = !climbDown;
  }
}
