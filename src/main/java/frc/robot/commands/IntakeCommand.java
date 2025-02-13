package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends Command {

    public LinearVelocity velocity;
    public LinearVelocity stop;
    public Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        velocity = LinearVelocity.ofBaseUnits(intake.getConstants().intakeVelocity, MetersPerSecond);
        stop = LinearVelocity.ofBaseUnits(0.0, MetersPerSecond);
    }

    @Override
    public void execute() {
        intake.setVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVelocity(stop);
    }
}

//test commit
