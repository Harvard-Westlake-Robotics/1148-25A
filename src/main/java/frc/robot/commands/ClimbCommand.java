package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Climb;

public class ClimbCommand extends Command {
  private Climb climb;
  private boolean deploy;
  private boolean climbDown;
  private double climbAngle = 120 * 5;
  private double zeroAngle = 450 - 270 - 30;
  private double stowAngle = 90 * 5;
  private boolean hasDeployed = false;

  public ClimbCommand() {
    this.addRequirements(Climb.getInstance());
    this.climb = Climb.getInstance();
    deploy = false;
    climbDown = false;
  }

  @Override
  public void initialize() {
    climb.goToAngle(climbAngle);
  }

  @Override
  public void execute() {
    if (climb.getLimitSwitch() && !deploy) {
      climbAngle = Climb.getInstance().getWristPosition();
      zeroAngle = Climb.getInstance().getWristPosition();
      stowAngle = Climb.getInstance().getWristPosition();
    } else {
      climbAngle = 93.5 * 5;
      stowAngle = 90 * 5;
      zeroAngle = 25 * 5 - 30;
    }

    if (climbDown) {
      deploy = false;
      climb.goToAngle(climbAngle);
      if (!climb.getLimitSwitch()) {
        climb.runVoltage(10.5);
      } else {
        climb.runVoltage(0.3);
      }
    } else if (deploy) {
      climb.goToAngle(zeroAngle);
      hasDeployed = true;

    } else {
      climb.goToAngle(stowAngle);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  public void deploy() {
    deploy = true;
  }

  public void climb() {
    if (hasDeployed)
      climbDown = true;
  }

  public void incrementClimb() {
    climbAngle++;
  }

  public void stow() {
    deploy = false;
    climbDown = false;
  }
}
