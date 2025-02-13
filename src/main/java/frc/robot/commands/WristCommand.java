package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;

public class WristCommand extends Command {

    public Angle angle;
    public Wrist wrist;

    public WristCommand(Wrist wrist, Angle angle) {
        this.wrist = wrist;
        addRequirements(wrist);
        this.angle = angle;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        wrist.goToAngle(angle);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
