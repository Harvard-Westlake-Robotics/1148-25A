package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommand extends Command {

  public double heightMeters;
  public Elevator elevator;

  public ElevatorCommand(double heightMeters) {
    this.elevator = Elevator.getInstance();
    addRequirements(elevator);
    this.heightMeters = heightMeters;
  }

  @Override
  public void initialize() {
    // elevator.goToHeight(0);
  }

  @Override
  public void execute() {
    elevator.goToHeight(heightMeters);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.goToHeight(0);
  }

  @Override
  public boolean isFinished() {
      return Math.abs(heightMeters - elevator.getHeight()) < 0.2;
  }

  public void setHeight(double heightMeters) {
    this.heightMeters = heightMeters;
  }
}
