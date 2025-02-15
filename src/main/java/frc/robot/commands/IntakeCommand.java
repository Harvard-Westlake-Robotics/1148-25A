package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;

public class IntakeCommand extends Command {

  public LinearVelocity velocity;
  public LinearVelocity stop;
  public Intake intake;
  public Wrist wrist;
  public double wristAngle;
  public Boolean sensor1Broken = null;

  public IntakeCommand(Intake intake) {
    this.intake = intake;
    this.wrist = null;
    addRequirements(intake);
    this.sensor1Broken = intake.getSensor1();
  }

  public IntakeCommand(Intake intake, Wrist wrist) {
    this.intake = intake;
    this.wrist = wrist;
    this.wristAngle = 2.2;
    addRequirements(intake, wrist);
  }

  @Override
  public void initialize() {
    velocity = LinearVelocity.ofBaseUnits(intake.getConstants().intakeVelocity, MetersPerSecond);
    stop = LinearVelocity.ofBaseUnits(0.0, MetersPerSecond);
    intake.setVelocity(velocity);
    if (wrist != null) {
      wrist.goToAngle(wristAngle);
    }
  }

  @Override
  public void execute() {
    if (sensor1Broken != null) {
      if (intake.getSensor1() == true && sensor1Broken) {
        this.cancel();
      } else if (intake.getSensor1() == false) {
        sensor1Broken = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.setVelocity(stop);
    if (wrist != null) {
      intake.runVoltage(1);
      if (Robot.robotContainer.wristIsDown) {
        wrist.goToAngle(0);
        intake.setVelocity(stop);
      }
    }
  }
}
// test commit
