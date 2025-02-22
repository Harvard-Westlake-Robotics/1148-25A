package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.CoralIntake;

public class CoralIntakeCommand extends Command {
  private LinearVelocity velocity;
  private final Boolean sensor1Broken;
  private Boolean sensor2Broken;

  public CoralIntakeCommand() {
    addRequirements(CoralIntake.getInstance());
    this.sensor1Broken = CoralIntake.getInstance().getSensor1();
    this.sensor2Broken = CoralIntake.getInstance().getSensor2();
  }

  @Override
  public void initialize() {
    velocity = LinearVelocity.ofBaseUnits(0.0, MetersPerSecond);
  }

  @Override
  public void execute() {
    if (sensor1Broken != null && sensor2Broken != null && Elevator.getInstance().getHeight() < 2) {
      if (CoralIntake.getInstance().getSensor1() == true
          && CoralIntake.getInstance().getSensor2() == false) {
        this.cancel();
      } else if (CoralIntake.getInstance().getSensor1() == false) {
        setVelocity(LinearVelocity.ofBaseUnits(10, MetersPerSecond));
        sensor2Broken = true;
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
    return false;
  }

  public void setVelocity(LinearVelocity velocity) {
    this.velocity = velocity;
  }
}
