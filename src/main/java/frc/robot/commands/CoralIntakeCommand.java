package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.CoralIntake;

public class CoralIntakeCommand extends Command {
  private LinearVelocity velocity;
  private boolean eject;

  public boolean isEject() {
    return eject;
  }

  public CoralIntakeCommand() {
    addRequirements(CoralIntake.getInstance());
    this.eject = false;
    velocity = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
  }

  public CoralIntakeCommand(double velocityMPS) {
    addRequirements(CoralIntake.getInstance());
    velocity = LinearVelocity.ofBaseUnits(velocityMPS, MetersPerSecond);
    this.eject = false;
  }

  @Override
  public void initialize() {
    // if (CoralIntake.getInstance().getCurrentCommand() != null
    // && CoralIntake.getInstance().getCurrentCommand() != this) {
    // CoralIntake.getInstance().getCurrentCommand().cancel();
    // }
    if (CoralIntake.getInstance().hasCoral() && eject == false) {
      velocity = LinearVelocity.ofBaseUnits(-0.01, MetersPerSecond);
    }
    CoralIntake.getInstance().setVelocity(velocity);
  }

  @Override
  public void execute() {
    // if shooting coral
    if (eject) {
      CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(100, MetersPerSecond));
      // if the intake is currently spinning- no coral in hold
    } else if (velocity.baseUnitMagnitude() > 0) {
      // if coral has reached sensor 1
      if (!CoralIntake.getInstance().getSensor1()) {
        // if coral has reached sensor 2 - middle chamber
        if (!CoralIntake.getInstance().getSensor2()) {
          CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(6, MetersPerSecond));
          // if coral has passed sensor 3 but has not reached sensor 2 - upper chamber
        } else if (CoralIntake.getInstance().getSensor3()) {
          CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(8, MetersPerSecond));
        }
        // if coral has passed sensor one- lower chamber
      } else if (!CoralIntake.getInstance().getSensor2()) {
        CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
        // if empty
      } else {
        CoralIntake.getInstance().setVelocity(velocity);
      }
      // if the intake is still but we dont have a coral loaded- unsure why it is here
    } else if (CoralIntake.getInstance().getSensor2()) {
      CoralIntake.getInstance().setVelocity(velocity);
    } else {
      CoralIntake.getInstance().setVelocity(velocity);
    }
  }

  @Override
  public void end(boolean interrupted) {
    //
    CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    // CoralIntake.getInstance().push(0);
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
