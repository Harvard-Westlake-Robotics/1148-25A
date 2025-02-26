package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.wrist.AlgaeWrist;

public class AlgaeIntakeCommand extends Command {
  private LinearVelocity velocity;
  private LinearVelocity velocity2 = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
  private final double wristAngle = 2.2;
  private Integer index;
  public boolean buttonPressed = false;

  public AlgaeIntakeCommand() {
    addRequirements(AlgaeIntake.getInstance(), AlgaeWrist.getInstance());
    this.index = 0;
  }

  @Override
  public void initialize() {
    velocity = LinearVelocity.ofBaseUnits(0.0, MetersPerSecond);
  }

  @Override
  public void execute() {
    if (index == 0) {
      velocity = LinearVelocity.ofBaseUnits(0.0, MetersPerSecond);
      AlgaeIntake.getInstance().setVelocity(velocity2);
      AlgaeWrist.getInstance().goToAngle(0);

    } else if (index == 1) {
      velocity = LinearVelocity.ofBaseUnits(Constants.AlgaeIntake.intakeVelocity, MetersPerSecond);
      AlgaeIntake.getInstance().setVelocity(velocity);
      AlgaeWrist.getInstance().goToAngle(wristAngle);
    } else if (index == 2) {
      velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
      if (AlgaeWrist.getInstance().getWristPosition() <= 0.8 + 0.1) {
        AlgaeIntake.getInstance().runVoltage(1);
      }
      AlgaeWrist.getInstance().goToAngle(0.8);
    } else if (index == 3) {
      AlgaeIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(-100, MetersPerSecond));
      AlgaeWrist.getInstance().goToAngle(0);
    } else {
      AlgaeIntake.getInstance().setVelocity(velocity);
    }
  }

  @Override
  public void end(boolean interrupted) {
    AlgaeIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    AlgaeIntake.getInstance().runVoltage(1);
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
