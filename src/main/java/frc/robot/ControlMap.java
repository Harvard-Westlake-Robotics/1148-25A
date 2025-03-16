package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
                    if (Elevator.getInstance().getHeight() > 15
                        && Elevator.getInstance().getHeight() < 16) {
                      RobotContainer.coralIntakeCommand.setVelocity(
                          LinearVelocity.ofBaseUnits(20, MetersPerSecond));
                    } else {
                      RobotContainer.coralIntakeCommand.setVelocity(
                          LinearVelocity.ofBaseUnits(30, MetersPerSecond));
                    }

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
                  RobotContainer.algaeIntakeCommand.index();
                  RobotContainer.algaeIntakeCommand.buttonPressed = true;
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  RobotContainer.algaeIntakeCommand.buttonPressed = false;
                  RobotContainer.algaeIntakeCommand.buttonPressed = false;
                }));
    // Elevator

    driver
        .R1()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (Elevator.getInstance().getHeight() > 1) {
                    RobotContainer.elevatorCommand.setHeight(ScoringLevel.L0);
                  } else {
                    RobotContainer.elevatorCommand.setHeight(ScoringLevel.L1);
                  }
                  // a way to recieve selected
                  // scoring level

                }));
    driver
        .cross()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.coralIntakeCommand.setEject(true);
                  RobotContainer.coralIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(-50, MetersPerSecond));
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  RobotContainer.coralIntakeCommand.setEject(false);
                  RobotContainer.coralIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                }));

    operator
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand.setHeight(ScoringLevel.L2);
                }));

    operator
        .leftTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand.setHeight(ScoringLevel.L1);
                }));

    operator
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand.setHeight(ScoringLevel.L3);
                }));
    operator
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand.setHeight(ScoringLevel.L4);
                }));

    operator
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand.setHeight(ScoringLevel.TOP_REMOVE);
                }));

    operator
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand.setHeight(ScoringLevel.BOTTOM_REMOVE);
                }));
    operator
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.elevatorCommand.setHeight(ScoringLevel.L0);
                }));

    // Climb Commands
    operator
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.hangCommand.deploy();
                }));

    operator
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.hangCommand.climb();
                }));

    SmartDashboard.putData(
        "Increment Climb",
        new InstantCommand(
            () -> {
              RobotContainer.hangCommand.incrementClimb();
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
            new Command() {
              @Override
              public void initialize() {
                NetworkCommunicator.getInstance().getTeleopCommand().updateCommands();
                NetworkCommunicator.getInstance().getTeleopCommand().schedule();
              }

              @Override
              public void end(boolean interrupted) {
                NetworkCommunicator.getInstance().getTeleopCommand().cancel();
                Elevator.getInstance().goToHeight(0);
                if (CoralIntake.getInstance().hasCoral()) {
                  CoralIntake.getInstance()
                      .setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                } else {
                  CoralIntake.getInstance()
                      .setVelocity(LinearVelocity.ofBaseUnits(4, MetersPerSecond));
                }
                Drive.getInstance().stop();
                if (Drive.getInstance().getCurrentCommand() != null) {
                  Drive.getInstance().getCurrentCommand().cancel();
                }
                if (CoralIntake.getInstance().getCurrentCommand() != null) {
                  CoralIntake.getInstance().getCurrentCommand().cancel();
                }
                if (Elevator.getInstance().getCurrentCommand() != null) {
                  Elevator.getInstance().getCurrentCommand().cancel();
                }
              }
            });
    // new SequentialCommandGroup(
    // new InstantCommand(
    // () -> {
    // NetworkCommunicator.getInstance().getTeleopCommand().updateCommands();
    // CoralIntake.getInstance().removeDefaultCommand();
    // Elevator.getInstance().removeDefaultCommand();
    // }),
    // NetworkCommunicator.getInstance().getTeleopCommand()));
  }
}
