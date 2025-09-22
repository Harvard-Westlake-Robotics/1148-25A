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
import frc.robot.commands.GroundIntakeCommand;
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
        .whileTrue(
            new InstantCommand(
                () -> {
                  if (CoralIntake.getInstance().hasCoral()) {
                    RobotContainer.coralIntakeCommand.setEject(true);
                    if (Elevator.getInstance().getHeight() > 14
                        && Elevator.getInstance().getHeight() < 17) {
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
        .onFalse(
            new InstantCommand(
                () -> {
                  if (CoralIntake.getInstance().hasCoral()) {
                    RobotContainer.coralIntakeCommand.setEject(false);
                    RobotContainer.coralIntakeCommand.setVelocity(
                        LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                  } else {
                    if (RobotContainer.coralIntakeCommand.isEject()) {
                      RobotContainer.coralIntakeCommand.setEject(false);
                      RobotContainer.coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                    } else {
                      RobotContainer.coralIntakeCommand.setVelocity(
                        LinearVelocity.ofBaseUnits(4, MetersPerSecond));
                    }
                  }
                }));

    driver
        .square()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.coralIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(20, MetersPerSecond));
                  RobotContainer.coralIntakeCommand.setEject(true);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  RobotContainer.coralIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                  RobotContainer.coralIntakeCommand.setEject(false);
                }));
    operator
        .povRight()
        .whileTrue(
            new InstantCommand(
                () -> {
                  RobotContainer.coralIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(20, MetersPerSecond));
                  RobotContainer.coralIntakeCommand.setEject(true);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  RobotContainer.coralIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                  RobotContainer.coralIntakeCommand.setEject(false);
                }));
    // Algae Intake
    driver
        .L1()
        .whileTrue(new GroundIntakeCommand(LinearVelocity.ofBaseUnits(6.5, MetersPerSecond)));

    driver
        .povUp()
        .whileTrue(new GroundIntakeCommand(LinearVelocity.ofBaseUnits(-6.5, MetersPerSecond)));
    driver
        .povLeft()
        .whileTrue(new GroundIntakeCommand(LinearVelocity.ofBaseUnits(-4, MetersPerSecond)));
    // Elevator

    driver
        .R1()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (Elevator.getInstance().getHeight() > 1) {
                    RobotContainer.elevatorCommand.setHeight(ScoringLevel.L0);
                  } else {
                    RobotContainer.elevatorCommand.setHeight(ScoringLevel.L4);
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
                // this.addRequirements(
                // CoralIntake.getInstance(), Drive.getInstance(), Elevator.getInstance());
                if (Drive.getInstance().getCurrentCommand() != null) {
                  Drive.getInstance().getCurrentCommand().cancel();
                }
                if (Elevator.getInstance().getCurrentCommand() != null) {
                  Elevator.getInstance().getCurrentCommand().cancel();
                }
                RobotContainer.coralIntakeCommand.setEject(false);
                NetworkCommunicator.getInstance().getTeleopCommand().updateCommands();
                NetworkCommunicator.getInstance().getTeleopCommand().schedule();
              }

              @Override
              public void end(boolean interrupted) {
                NetworkCommunicator.getInstance().getTeleopCommand().cancel();
                Elevator.getInstance().goToHeight(0);
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
