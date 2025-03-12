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
    climb.goToAngle(54);
  }

  @Override
  public void execute() {

    if (climbDown) {
      deploy = false;
      climb.goToAngle(58);
    } else if (deploy) {
      climb.goToAngle(0);

    } else {
      climb.goToAngle(54);
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
