package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.CoralIntake;

public class CoralIntakeCommand extends Command {
  private LinearVelocity velocity;
  private boolean eject;

  public CoralIntakeCommand() {
    addRequirements(CoralIntake.getInstance());
    this.eject = false;
    velocity = LinearVelocity.ofBaseUnits(4, MetersPerSecond);
  }

  public CoralIntakeCommand(double velocityMPS) {
    addRequirements(CoralIntake.getInstance());
    velocity = LinearVelocity.ofBaseUnits(velocityMPS, MetersPerSecond);
    this.eject = false;
  }

  @Override
  public void initialize() {
    CoralIntake.getInstance().setVelocity(velocity);
  }

  @Override
  public void execute() {
    if (eject) {
      CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(100, MetersPerSecond));
    } else if (velocity.baseUnitMagnitude() > 0) {
      if (CoralIntake.getInstance().getSensor1() == false) {
        if (CoralIntake.getInstance().getSensor3() == true){
          velocity = LinearVelocity.ofBaseUnits(12, MetersPerSecond);
          CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(12, MetersPerSecond));
        }
      } else if (CoralIntake.getInstance().getSensor2() == false) {
        velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
        CoralIntake.getInstance().setVelocity(velocity);
        this.cancel();
      } else {
        CoralIntake.getInstance().setVelocity(velocity);
      }
    } else {
      CoralIntake.getInstance().setVelocity(velocity);
    }
  }

  @Override
  public void end(boolean interrupted) {
    CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
  }

  @Override
  public boolean isFinished() {
    if (!eject) {
      return CoralIntake.getInstance().hasCoral();
    } else {
      return CoralIntake.getInstance().getSensor2();
    }
  }

  public void setVelocity(LinearVelocity velocity) {
    this.velocity = velocity;
  }

  public void setEject(boolean eject) {
    this.eject = eject;
  }
}
