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
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GroundIntakeCommand;
import frc.robot.commands.RaiseElevatorCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ScoreCommand.ScoringLevel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LEDs.LED;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.wrist.AlgaeWrist;
import frc.robot.subsystems.wrist.Climb;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
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
  public final CoralIntake coralIntake;
  private final Elevator elevator;
  private final AlgaeWrist intakeWrist;
  private final Climb hangWrist;
  public static boolean isDriftModeActive = false;

  // Controller
  public final CommandXboxController operator = new CommandXboxController(1);
  public final CommandPS5Controller driver = new CommandPS5Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Command> preAutoChooser;

  public static ScoreCommand elevatorCommand = new ScoreCommand(ScoringLevel.L0);
  public static GroundIntakeCommand algaeIntakeCommand;
  public static CoralIntakeCommand coralIntakeCommand;
  public static ClimbCommand hangCommand;

  public static SwerveDriveSimulation driveSimulation = null;

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
            new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
            new ModuleIOTalonFXReal(TunerConstants.FrontRight),
            new ModuleIOTalonFXReal(TunerConstants.BackLeft),
            new ModuleIOTalonFXReal(TunerConstants.BackRight),
            pose -> {
            });
        this.algaeIntake = AlgaeIntake.getInstance();
        this.coralIntake = CoralIntake.getInstance();
        this.elevator = Elevator.getInstance();
        this.intakeWrist = AlgaeWrist.getInstance();
        this.hangWrist = Climb.getInstance();
        algaeIntakeCommand = new GroundIntakeCommand();
        coralIntakeCommand = new CoralIntakeCommand();
        hangCommand = new ClimbCommand();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive = new Drive(
            new GyroIOSim(driveSimulation.getGyroSimulation()),
            new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
            new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
            new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
            new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
            driveSimulation::setSimulationWorldPose);
        this.algaeIntake = AlgaeIntake.getInstance();
        this.coralIntake = CoralIntake.getInstance();
        this.elevator = Elevator.getInstance();
        this.intakeWrist = AlgaeWrist.getInstance();
        this.hangWrist = Climb.getInstance();
        algaeIntakeCommand = new GroundIntakeCommand();
        coralIntakeCommand = new CoralIntakeCommand();
        hangCommand = new ClimbCommand();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOTalonFX(TunerConstants.FrontLeft) {
            },
            new ModuleIOTalonFX(TunerConstants.FrontRight) {
            },
            new ModuleIOTalonFX(TunerConstants.BackLeft) {
            },
            new ModuleIOTalonFX(TunerConstants.BackRight) {
            },
            pose -> {
            });
        this.algaeIntake = AlgaeIntake.getInstance();
        this.coralIntake = CoralIntake.getInstance();
        this.elevator = Elevator.getInstance();
        this.intakeWrist = AlgaeWrist.getInstance();
        this.hangWrist = Climb.getInstance();
        algaeIntakeCommand = new GroundIntakeCommand();
        coralIntakeCommand = new CoralIntakeCommand();
        hangCommand = new ClimbCommand();
        break;
    }

    NamedCommands.registerCommand("IntakeCoral", new CoralIntakeCommand(30).withTimeout(4));
    NamedCommands.registerCommand("ScoreL4", new RaiseElevatorCommand(ScoringLevel.L4));
    NamedCommands.registerCommand("ScoreL3", new RaiseElevatorCommand(ScoringLevel.L3));
    NamedCommands.registerCommand("ScoreL2", new RaiseElevatorCommand(ScoringLevel.L2));
    NamedCommands.registerCommand("ScoreL1", new RaiseElevatorCommand(ScoringLevel.L1));
    NamedCommands.registerCommand("ScoreAlgae", new CoralIntakeCommand(-30).withTimeout(4));
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

    // Add drift compensation calibration
    autoChooser.addOption(
        "Drive Drift Compensation Calibration", DriveCommands.driftCompensationCalibration(drive));

    // Add traction control test
    autoChooser.addOption("Traction Control Test", DriveCommands.tractionControlTest(drive));

    // Configure the button bindings
    configureButtonBindings();
    LED.getInstance().Color(255, 0, 0);
    try {
      pathfindL = PathPlannerPath.fromPathFile("Push");
    } catch (Exception e) {
    }
    preAutoChooser = new LoggedDashboardChooser<>("Pre Auto Choices", new SendableChooser<>());
    preAutoChooser.addDefaultOption("None", Commands.none());
    preAutoChooser.addOption("Push", AutoBuilder.followPath(pathfindL));
    SmartDashboard.putData("Pre Auto Chooser", preAutoChooser.getSendableChooser());
    Command stowCommand = new Command() {
      @Override
      public void initialize() {
        hangCommand.stow();
      }
    };
    stowCommand.runsWhenDisabled();
    SmartDashboard.putData("Stow Hang", stowCommand.withTimeout(0.1));
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
    // Dynamic default command based on drift mode state
    drive.setDefaultCommand(
        // Regular joystick drive with rotation and xy
        Commands.either(
            // Drift mode: Use R2 as throttle and left stick X as steering
            DriveCommands.carDriftDrive(
                drive,
                () -> driver.getR2Axis(), // R2 trigger for throttle
                () -> -driver.getLeftX() // Left stick X for steering (inverted)
            ),
            // Normal mode: Standard swerve controls
            DriveCommands.joystickDrive(
                drive,
                () -> -driver.getLeftY(), // Forward/backward
                () -> -driver.getLeftX(), // Left/right strafe
                () -> -driver.getRightX() // Rotation
            ),
            () -> RobotContainer.isDriftModeActive // Condition to check
        )
    // // Just xy, pointing at a vision target
    );
    // Get robot position in x and y
    // double robotX = drive.getPose().getX();
    // double robotY = drive.getPose().getY();
    // Translation2d robotTranslation = drive.getPose().getTranslation();
    // // Get reef position in x and y (stored in constant)
    // // x is the first element, y is the second element
    // double[] reefCoords = Drive.getInstance().getReefPositions();
    // // Calculate delta x and y
    // double deltaX = reefCoords[0] - robotX;
    // double deltaY = reefCoords[1] - robotY;
    // // Find the angle, put that into drive at angle
    // double angle = Math.atan2(deltaX, deltaY);
    // drive.setDefaultCommand(
    // DriveCommands.joystickDriveAtAngle(
    // drive,
    // () -> -driver.getLeftY(), // Forward/backward
    // () -> -driver.getLeftX(), // Left/right strafe
    // () ->
    // new Rotation2d(
    // angle // Rotation
    // )));

    elevatorCommand = new ScoreCommand(ScoringLevel.L0);
    elevator.setDefaultCommand(elevatorCommand);

    algaeIntakeCommand = new GroundIntakeCommand(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    algaeIntake.setDefaultCommand(algaeIntakeCommand);
    hangCommand = new ClimbCommand();
    hangWrist.setDefaultCommand(hangCommand);

    // Reset gyro / odometry
    final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
        ? () -> drive.setPose(
            driveSimulation
                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
        // simulation
        : () -> drive.setPose(
            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    driver.povCenter().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // // Add drift mode toggle to the driver's right bumper button
    // operator.povLeft().onTrue(DriveCommands.toggleDriftMode(drive));

    // // Add traction control toggle (disabled by default for testing)
    // operator.povRight().onTrue(DriveCommands.toggleTractionControl(drive));

    // // Add traction control test command (hold for monitoring)
    // operator.leftBumper().whileTrue(DriveCommands.tractionControlTest(drive));

    ControlMap.getInstance().configurePreset1(operator, driver);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (coralIntakeCommand != null)
      coralIntakeCommand.cancel();
    if (CoralIntake.getInstance().getDefaultCommand() != null)
      CoralIntake.getInstance().removeDefaultCommand();
    return autoChooser.get();
    // return preAutoChooser
    // .get()
    // .andThen(
    // new Command() {}.withTimeout(0.3)
    // .andThen(NetworkCommunicator.getInstance().getCustomAuto()));
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM)
      return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM)
      return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
