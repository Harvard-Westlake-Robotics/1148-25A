package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ScoreCommand.ScoringLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.NetworkCommunicator;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.wrist.Climb;

public class ControlMap {
    private RobotContainer robotContainer = Robot.robotContainer;
    private static ControlMap instance;

    public static ControlMap getInstance() {
        if (instance == null) {
            instance = new ControlMap();
        }
        return instance;
    }

    private ControlMap() {

    }

    public void configurePreset1(CommandXboxController operator, CommandPS5Controller driver) {
        // Reset gyro to 0° when B button is pressed
    operator
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        Drive.getInstance().setPose(
                            new Pose2d(Drive.getInstance().getPose().getTranslation(), new Rotation2d())),
                    Drive.getInstance())
                .ignoringDisable(true));

    // Intake commands

    // Coral Intake
    driver
        .L2()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (CoralIntake.getInstance().hasCoral()) {
                    robotContainer.coralIntakeCommand.setEject(true);
                    robotContainer.coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(50, MetersPerSecond));
                  } else {
                    robotContainer.coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(20, MetersPerSecond));
                    robotContainer.coralIntakeCommand.setEject(false);
                  }
                }))
        .whileTrue(robotContainer.coralIntakeCommand)
        .onFalse(
            new InstantCommand(
                () -> {
                  if (CoralIntake.getInstance().hasCoral()) {
                    robotContainer.coralIntakeCommand.setEject(false);
                    robotContainer.coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                  } else {
                    robotContainer.coralIntakeCommand.setEject(false);
                    robotContainer.coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(4, MetersPerSecond));
                  }
                }));

    driver
        .square()
        .toggleOnTrue(
            new InstantCommand(
                () -> {
                  robotContainer.coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(-50, MetersPerSecond));
                }))
        .toggleOnFalse(
            new InstantCommand(
                () -> {
                  robotContainer.coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(4, MetersPerSecond));
                }));
    // Algae Intake
    driver
        .circle()
        .onTrue(
            new InstantCommand(
                () -> {
                  robotContainer.algaeIntakeCommand.index();
                  robotContainer.algaeIntakeCommand.buttonPressed = true;
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  robotContainer.algaeIntakeCommand.buttonPressed = false;
                }));
    // Elevator
    operator
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  robotContainer.elevatorCommand = new ScoreCommand(ScoringLevel.L2);
                  Elevator.getInstance().setDefaultCommand(robotContainer.elevatorCommand);
                }));

    operator
        .leftTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  robotContainer.elevatorCommand = new ScoreCommand(ScoringLevel.L1);
                  Elevator.getInstance().setDefaultCommand(robotContainer.elevatorCommand);
                }));

    operator
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  robotContainer.elevatorCommand = new ScoreCommand(ScoringLevel.L3);
                  Elevator.getInstance().setDefaultCommand(robotContainer.elevatorCommand);
                }));
    operator
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  robotContainer.elevatorCommand = new ScoreCommand(ScoringLevel.L4);
                  Elevator.getInstance().setDefaultCommand(robotContainer.elevatorCommand);
                }));

    operator
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  robotContainer.elevatorCommand = new ScoreCommand(ScoringLevel.TOP_REMOVE);
                  Elevator.getInstance().setDefaultCommand(robotContainer.elevatorCommand);
                }));

    operator
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  robotContainer.elevatorCommand = new ScoreCommand(ScoringLevel.BOTTOM_REMOVE);
                  Elevator.getInstance().setDefaultCommand(robotContainer.elevatorCommand);
                }));
    operator
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> {
                  robotContainer.elevatorCommand = new ScoreCommand(ScoringLevel.L0);
                  Elevator.getInstance().setDefaultCommand(robotContainer.elevatorCommand);
                }));

    // Climb Commands
    operator
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  robotContainer.hangCommand.deploy();
                }));

    operator
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  robotContainer.hangCommand.climb();
                }));

    // Pathfinding Commands
    driver
        .L1()
        .whileTrue(
            AutoBuilder.pathfindThenFollowPath(
                NetworkCommunicator.getInstance().getSelectedSourcePath(), Drive.PP_CONSTRAINTS).andThen(new CoralIntakeCommand(20)));
    driver
        .R1()
        .whileTrue(
            AutoBuilder.pathfindThenFollowPath(
                NetworkCommunicator.getInstance().getSelectedReefPath(), Drive.PP_CONSTRAINTS).andThen(new AutoScoreCommand(ScoringLevel.L4)));

    driver
        .cross()
        .onTrue(
            new InstantCommand(
                () -> {
                  robotContainer.coralIntakeCommand.setEject(true);
                  robotContainer.coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(2000, MetersPerSecond));
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  robotContainer.coralIntakeCommand.setEject(false);
                  robotContainer.coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(4, MetersPerSecond));
                }));
    }
}
