package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.wrist.AlgaeWrist;

public class GroundIntakeCommand extends Command {
  private LinearVelocity velocity;
  private LinearVelocity velocity2 = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
  private final double wristAngle = 4.88;
  private Integer index;
  public boolean buttonPressed = false;

  public GroundIntakeCommand() {
    addRequirements(AlgaeIntake.getInstance(), AlgaeWrist.getInstance());
    this.index = 0;
    velocity = LinearVelocity.ofBaseUnits(0.0, MetersPerSecond);
  }

  public GroundIntakeCommand(LinearVelocity velocity) {
    addRequirements(AlgaeIntake.getInstance(), AlgaeWrist.getInstance());
    this.index = 0;
    this.velocity = velocity;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (velocity.baseUnitMagnitude() > 0) {
      AlgaeIntake.getInstance().setVelocity(velocity);
      AlgaeWrist.getInstance().goToAngle(wristAngle);
    } else if (velocity.baseUnitMagnitude() == -4) {
      AlgaeIntake.getInstance().setVelocity(velocity);
      AlgaeWrist.getInstance().goToAngle(3.5);
    } else if (velocity.baseUnitMagnitude() < 0) {
      AlgaeIntake.getInstance().setVelocity(velocity);
      AlgaeWrist.getInstance().goToAngle(1);
    } else {
      AlgaeIntake.getInstance().setVelocity(velocity);
      AlgaeWrist.getInstance().goToAngle(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    AlgaeIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    AlgaeWrist.getInstance().goToAngle(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public void index() {
    if (index >= 3) {
      index = 0;
    } else {
      index++;
    }
  }

  public void setVelocity(LinearVelocity velocity) {
    this.velocity = velocity;
  }

  public void outtake() {
    this.index = 3;
    this.velocity =
        LinearVelocity.ofBaseUnits(-Constants.AlgaeIntake.intakeVelocity, MetersPerSecond);
  }
}
