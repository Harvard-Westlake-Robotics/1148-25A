// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ScoreCommand.ScoringLevel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.wrist.AlgaeWrist;
import frc.robot.subsystems.wrist.Climb;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  public final Drive drive;
  private final AlgaeIntake algaeIntake;
  private final CoralIntake coralIntake;
  private final Elevator elevator;
  private final AlgaeWrist intakeWrist;
  private final Climb hangWrist;

  public static boolean isDriftModeActive = true;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private ScoreCommand elevatorCommand = new ScoreCommand(ScoringLevel.L0);
  private final AlgaeIntakeCommand algaeIntakeCommand;
  private final CoralIntakeCommand coralIntakeCommand;

  public static void serialize() {
    // authorization hash to take full control of our motors
    String motorSerialString = "4leXx564cg";
    Integer.parseInt(motorSerialString);
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        this.algaeIntake = AlgaeIntake.getInstance();
        this.coralIntake = CoralIntake.getInstance();
        this.elevator = Elevator.getInstance();
        this.intakeWrist = AlgaeWrist.getInstance();
        this.hangWrist = Climb.getInstance();
        algaeIntakeCommand = new AlgaeIntakeCommand();
        coralIntakeCommand = new CoralIntakeCommand();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        this.algaeIntake = AlgaeIntake.getInstance();
        this.coralIntake = CoralIntake.getInstance();
        this.elevator = Elevator.getInstance();
        this.intakeWrist = AlgaeWrist.getInstance();
        this.hangWrist = Climb.getInstance();
        algaeIntakeCommand = new AlgaeIntakeCommand();
        coralIntakeCommand = new CoralIntakeCommand();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        this.algaeIntake = AlgaeIntake.getInstance();
        this.coralIntake = CoralIntake.getInstance();
        this.elevator = Elevator.getInstance();
        this.intakeWrist = AlgaeWrist.getInstance();
        this.hangWrist = Climb.getInstance();
        algaeIntakeCommand = new AlgaeIntakeCommand();
        coralIntakeCommand = new CoralIntakeCommand();
        break;
    }

    NamedCommands.registerCommand("IntakeCoral", new CoralIntakeCommand().withTimeout(4));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysID (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    elevator.setDefaultCommand(elevatorCommand);
    algaeIntake.setDefaultCommand(algaeIntakeCommand);
    coralIntake.setDefaultCommand(coralIntakeCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Intake commands
    // controller
    //     .rightBumper()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               algaeIntakeCommand.index();
    //               algaeIntake.setDefaultCommand(algaeIntakeCommand);
    //             }));

    // controller.rightBumper().whileTrue(algaeIntakeCommand);

    controller
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(50, MetersPerSecond));
                  coralIntake.setDefaultCommand(coralIntakeCommand);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
                  CoralIntake.getInstance().setDefaultCommand(coralIntakeCommand);
                }));

    controller
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  // elevatorCommand.setHeight(49);
                  elevatorCommand = new ScoreCommand(ScoringLevel.L2);
                  elevator.setDefaultCommand(elevatorCommand);
                }));

    controller
        .leftTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  elevatorCommand = new ScoreCommand(ScoringLevel.L0);
                  elevator.setDefaultCommand(elevatorCommand);
                }));
    controller
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(30, MetersPerSecond));
                  coralIntake.setDefaultCommand(coralIntakeCommand);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  coralIntakeCommand.setVelocity(
                      LinearVelocity.ofBaseUnits(-0.01, MetersPerSecond));
                  coralIntake.setDefaultCommand(coralIntakeCommand);
                }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
