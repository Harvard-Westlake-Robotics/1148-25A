package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ScoreCommand.ScoringLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.NetworkCommunicator;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.CoralIntake;

public class ControlMap {
  private static ControlMap instance;

  public static ControlMap getInstance() {
    if (instance == null) {
      instance = new ControlMap();
    }
    return instance;
  }

  private ControlMap() {}

  public void configurePreset1(CommandXboxController operator, CommandPS5Controller driver) {
    // Reset gyro to 0° when B button is pressed
    operator
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        Drive.getInstance()
                            .setPose(
                                new Pose2d(
                                    Drive.getInstance().getPose().getTranslation(),
                                    new Rotation2d())),
                    Drive.getInstance())
                .ignoringDisable(true));

    // Intake commands

    // Coral Intake
    driver
        .R2()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (CoralIntake.getInstance().hasCoral()) {
                    RobotContainer.coralIntakeCommand.setEject(true);
                    RobotContainer.coralIntakeCommand.setVelocity(
                        LinearVelocity.ofBaseUnits(50, MetersPerSecond));
                  } else {
                    RobotContainer.coralIntakeCommand.setVelocity(
                        LinearVelocity.ofBaseUnits(20, MetersPerSecond));
                    RobotContainer.coralIntakeCommand.setEject(false);
                  }
                }))
        .whileTrue(RobotContainer.coralIntakeCommand)
        .onFalse(
            new InstantCommand(
                () -> {
                  if (CoralIntake.getInstance().hasCoral()) {
                    RobotContainer.coralIntakeCommand.setEject(false);
                    RobotContainer.coralIntakeCommand.setVelocity(
                        LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                  } else {
                    RobotContainer.coralIntakeCommand.setEject(false);
                    RobotContainer.coralIntakeCommand.setVelocity(
                        LinearVelocity.ofBaseUnits(4, MetersPerSecond));
                  }
                }));

    driver
        .square()
        .toggleOnTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.coralIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(-50, MetersPerSecond));
                }))
        .toggleOnFalse(
            new InstantCommand(
                () -> {
                  RobotContainer.coralIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(4, MetersPerSecond));
                }));
    // Algae Intake
    driver
        .L1()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (Elevator.getInstance().getHeight() == 53.40) {
                    RobotContainer.coralIntakeCommand.setEject(true);
                    RobotContainer.coralIntakeCommand.setVelocity(
                        LinearVelocity.ofBaseUnits(-50, MetersPerSecond));
                  } else {
                    RobotContainer.algaeIntakeCommand.index();
                    RobotContainer.algaeIntakeCommand.buttonPressed = true;
                  }
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  if (Elevator.getInstance().getHeight() == 53.40) {
                    RobotContainer.coralIntakeCommand.setEject(false);
                    RobotContainer.coralIntakeCommand.setVelocity(
                        LinearVelocity.ofBaseUnits(4, MetersPerSecond));
                  } else {
                    RobotContainer.algaeIntakeCommand.buttonPressed = false;
                  }
                  RobotContainer.algaeIntakeCommand.buttonPressed = false;
                }));
    // Elevator

    driver
        .R1()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand =
                      new ScoreCommand(ScoringLevel.L1); // TODO: Chase needs to implement
                  // a way to recieve selected
                  // scoring level
                  Elevator.getInstance().setDefaultCommand(RobotContainer.elevatorCommand);
                }));

    operator
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand = new ScoreCommand(ScoringLevel.L2);
                  Elevator.getInstance().setDefaultCommand(RobotContainer.elevatorCommand);
                }));

    operator
        .leftTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand = new ScoreCommand(ScoringLevel.L1);
                  Elevator.getInstance().setDefaultCommand(RobotContainer.elevatorCommand);
                }));

    operator
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand = new ScoreCommand(ScoringLevel.L3);
                  Elevator.getInstance().setDefaultCommand(RobotContainer.elevatorCommand);
                }));
    operator
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand = new ScoreCommand(ScoringLevel.L4);
                  Elevator.getInstance().setDefaultCommand(RobotContainer.elevatorCommand);
                }));

    operator
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand = new ScoreCommand(ScoringLevel.TOP_REMOVE);
                  Elevator.getInstance().setDefaultCommand(RobotContainer.elevatorCommand);
                }));

    operator
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand = new ScoreCommand(ScoringLevel.BOTTOM_REMOVE);
                  Elevator.getInstance().setDefaultCommand(RobotContainer.elevatorCommand);
                }));
    operator
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand = new ScoreCommand(ScoringLevel.L0);
                  Elevator.getInstance().setDefaultCommand(RobotContainer.elevatorCommand);
                }));

    // Climb Commands
    operator
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.hangCommand.deploy();
                }));

    driver
        .cross()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.hangCommand.climb();
                }));

    // Pathfinding Commands
    // driver
    // .L2()
    // .whileTrue(
    // AutoBuilder.pathfindThenFollowPath(
    // NetworkCommunicator.getInstance().getSelectedSourcePath(),
    // Drive.PP_CONSTRAINTS)
    // .andThen(new CoralIntakeCommand(20)));
    driver
        .L2()
        .whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> NetworkCommunicator.getInstance().getTeleopCommand().updateCommands()),
                NetworkCommunicator.getInstance().getTeleopCommand()));
  }
}
