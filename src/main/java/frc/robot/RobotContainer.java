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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RaiseElevatorCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ScoreCommand.ScoringLevel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDs.LED;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.NetworkCommunicator;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.wrist.AlgaeWrist;
import frc.robot.subsystems.wrist.Climb;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Paths
  private PathPlannerPath pathfindL;
  private PathPlannerPath pathfindSource;

  // Subsystems
  public final Drive drive;
  private final AlgaeIntake algaeIntake;
  private final CoralIntake coralIntake;
  private final Elevator elevator;
  private final AlgaeWrist intakeWrist;
  private final Climb hangWrist;
  public static boolean isDriftModeActive = false;

  // Controller
  public final CommandXboxController operator = new CommandXboxController(1);
  public final CommandPS5Controller driver = new CommandPS5Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public static ScoreCommand elevatorCommand = new ScoreCommand(ScoringLevel.L0);
  public static AlgaeIntakeCommand algaeIntakeCommand;
  public static CoralIntakeCommand coralIntakeCommand;
  public static ClimbCommand hangCommand;

  public boolean elevatorDeployed = false;

  public static void serialize() {
    // authorization hash to take full control of our motors
    String motorSerialString = "4leXx564cg";
    Integer.parseInt(motorSerialString);
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
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
        hangCommand = new ClimbCommand();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIO() {
            },
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
        hangCommand = new ClimbCommand();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        this.algaeIntake = AlgaeIntake.getInstance();
        this.coralIntake = CoralIntake.getInstance();
        this.elevator = Elevator.getInstance();
        this.intakeWrist = AlgaeWrist.getInstance();
        this.hangWrist = Climb.getInstance();
        algaeIntakeCommand = new AlgaeIntakeCommand();
        coralIntakeCommand = new CoralIntakeCommand();
        hangCommand = new ClimbCommand();
        break;
    }

    NamedCommands.registerCommand("IntakeCoral", new CoralIntakeCommand(30).withTimeout(4));
    NamedCommands.registerCommand("ScoreL4", new RaiseElevatorCommand(ScoringLevel.L4));
    NamedCommands.registerCommand("ScoreL3", new RaiseElevatorCommand(ScoringLevel.L3));
    NamedCommands.registerCommand("ScoreL2", new RaiseElevatorCommand(ScoringLevel.L2));
    NamedCommands.registerCommand("ScoreL1", new RaiseElevatorCommand(ScoringLevel.L1));
    NamedCommands.registerCommand("ElevatorDown", new ScoreCommand(ScoringLevel.L0));
    NamedCommands.registerCommand("AutoScore L4", new AutoScoreCommand(ScoringLevel.L4));
    NamedCommands.registerCommand("AutoScore L3", new AutoScoreCommand(ScoringLevel.L3));
    NamedCommands.registerCommand("AutoScore L2", new AutoScoreCommand(ScoringLevel.L2));
    NamedCommands.registerCommand("AutoScore L1", new AutoScoreCommand(ScoringLevel.L1));

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

    autoChooser.addOption(
        "Elevator SysId (Quasistatic Forward)",
        elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Elevator SysId (Quasistatic Reverse)",
        elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Elevator SysId (Dynamic Forward)", elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Elevator SysID (Dynamic Reverse)", elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    LED.getInstance().Color(0, 255, 0);
    try {
      pathfindL = PathPlannerPath.fromPathFile("Push");
    } catch (Exception e) {
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    elevatorCommand = new ScoreCommand(ScoringLevel.L0);
    elevator.setDefaultCommand(elevatorCommand);
    coralIntakeCommand = new CoralIntakeCommand(4);
    coralIntake.setDefaultCommand(coralIntakeCommand);
    algaeIntakeCommand = new AlgaeIntakeCommand();
    algaeIntake.setDefaultCommand(algaeIntakeCommand);
    hangCommand = new ClimbCommand();
    hangWrist.setDefaultCommand(hangCommand);
    ControlMap.getInstance().configurePreset1(operator, driver);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return NetworkCommunicator.getInstance().getCustomAuto();
  }
}
