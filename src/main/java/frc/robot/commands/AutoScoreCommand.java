package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ScoreCommand.ScoringLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.CoralIntake;
import org.littletonrobotics.junction.Logger;

/**
 * Command to automatically score game pieces at specified heights. This command
 * handles both the
 * elevator movement and drive positioning. It includes safety checks, timeouts,
 * and error handling.
 */
public class AutoScoreCommand extends Command {
  // Constants for position and timing
  public static double POSITION_TOLERANCE = 0.02; // meters
  public static double ROTATION_TOLERANCE = 0.40; // degrees
  private static final double ELEVATOR_TOLERANCE = 0.8; // meters
  private static final double SCORING_DELAY_TICKS = 2;
  private static final double SCORING_VELOCITY = 100.0; // meters per second
  private static final double DEFAULT_VELOCITY = 6.0; // meters per second
  private static final double COMMAND_TIMEOUT = 10.0; // seconds

  // PID controller constants
  public static double X_PID_P = 18.7;
  public static double X_PID_D = 0.05;
  public static double Y_PID_P = 18.7;
  public static double Y_PID_D = 0.05;
  public static double THETA_PID_P = 9.9;
  public static double THETA_PID_D = 0.1;
  private static final double MAX_VELOCITY = 2.0;
  private static final double MAX_ACCELERATION = 2.5;
  private static final double MAX_ANGULAR_VELOCITY = 2.0;
  private static final double MAX_ANGULAR_ACCELERATION = 2.5;

  private final ScoringLevel level;
  private double targetHeight = 0.0;
  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;
  private final Pose2d endPose;
  private int tickCounter = 0;
  private final Timer timeoutTimer;
  private boolean hasElevatorError = false;

  /**
   * Creates a new AutoScoreCommand for scoring at a specific level.
   *
   * @param level The scoring level to move to
   */
  public AutoScoreCommand(ScoringLevel level) {
    this.addRequirements(CoralIntake.getInstance(), Elevator.getInstance());
    this.level = level;
    this.timeoutTimer = new Timer();

    // Initialize controllers with constants
    this.xController = new ProfiledPIDController(
        X_PID_P, 0.00, X_PID_D, new Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    this.yController = new ProfiledPIDController(
        Y_PID_P, 0.00, Y_PID_D, new Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    this.thetaController = new ProfiledPIDController(
        THETA_PID_P,
        0,
        THETA_PID_D,
        new Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION));

    // Set up controllers
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);
    thetaController.setTolerance(Math.toRadians(ROTATION_TOLERANCE));

    // Set target height based on level
    setTargetHeight(level);

    // Initialize endPose as null since we're not using path following
    this.endPose = null;
  }

  /**
   * Creates a new AutoScoreCommand with path following capability.
   *
   * @param level The scoring level to move to
   * @param path  The path to follow to the scoring position
   */
  public AutoScoreCommand(ScoringLevel level, PathPlannerPath path) {
    this.addRequirements(CoralIntake.getInstance(), Elevator.getInstance(), Drive.getInstance());
    this.level = level;
    this.timeoutTimer = new Timer();

    // Cancel any existing intake command
    if (CoralIntake.getInstance().getCurrentCommand() != null) {
      CoralIntake.getInstance().getCurrentCommand().cancel();
    }
    if (Drive.getInstance().getCurrentCommand() != null) {
      Drive.getInstance().getCurrentCommand().cancel();
    }
    // RobotContainer.coralIntakeCommand.setVelocity(LinearVelocity.ofBaseUnits(0,
    // MetersPerSecond));

    // Initialize controllers with constants
    this.xController = new ProfiledPIDController(
        X_PID_P, 0, X_PID_D, new Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    this.yController = new ProfiledPIDController(
        Y_PID_P, 0, Y_PID_D, new Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    this.thetaController = new ProfiledPIDController(
        THETA_PID_P,
        0,
        THETA_PID_D,
        new Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION));

    // Set up controllers
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);
    thetaController.setTolerance(Math.toRadians(ROTATION_TOLERANCE));

    // Set target height based on level
    setTargetHeight(level);

    // Safely get end pose from path
    if (path != null) {
      this.endPose = new Pose2d(
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
              ? path.getPathPoses()
                  .get(path.getPathPoses().size() - 1)
                  .getTranslation()
                  .plus(new Translation2d(0.04, path.getGoalEndState().rotation()))
              : path.flipPath()
                  .getPathPoses()
                  .get(path.getPathPoses().size() - 1)
                  .getTranslation()
                  .plus(new Translation2d(0.04, path.flipPath().getGoalEndState().rotation())),
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
              ? path.getGoalEndState().rotation()
              : path.flipPath().getGoalEndState().rotation());
    } else {
      throw new IllegalArgumentException("Path must contain at least 3 poses");
    }
    Logger.recordOutput("RealOutputs/PIDEndPose", endPose);

    // Reset controllers to current position
    Pose2d currentPose = Drive.getInstance().getPose();
    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  /**
   * Sets the target height based on the scoring level.
   *
   * @param level The scoring level to set height for
   */
  private void setTargetHeight(ScoringLevel level) {
    switch (level) {
      case L1:
        targetHeight = 15.5;
        break;
      case L2:
        targetHeight = 21.3;
        break;
      case L3:
        targetHeight = 32.68;
        break;
      case L4:
        targetHeight = 53.40;
        break;
      case TOP_REMOVE:
        targetHeight = 19.12;
        break;
      case BOTTOM_REMOVE:
        targetHeight = 7.60;
        break;
      default:
        targetHeight = 0;
        break;
    }
  }

  @Override
  public void initialize() {
    xController.reset(Drive.getInstance().getPose().getX());
    yController.reset(Drive.getInstance().getPose().getY());
    thetaController.reset(Drive.getInstance().getPose().getRotation().getRadians());
    Drive.getInstance().setSdMultiplier(8);
    // Reset command state
    tickCounter = 0;
    hasElevatorError = false;
    timeoutTimer.reset();
    timeoutTimer.start();

    // Set initial positions and velocities
    setTargetHeight(level);
    CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    Elevator.getInstance().goToHeight(targetHeight);
  }

  @Override
  public void execute() {

    // Check for timeout
    if (timeoutTimer.get() > COMMAND_TIMEOUT) {
      this.cancel();
      return;
    }

    // Check for elevator errors
    double currentHeight = Elevator.getInstance().getHeight();
    if (currentHeight < 0) {
      hasElevatorError = true;
      this.cancel();
      return;
    }

    // Move elevator to target height
    Elevator.getInstance().goToHeight(targetHeight);

    // Handle path following if endPose is set
    if (endPose != null) {
      handlePathFollowing();
    } else {
      handleScoring();
    }
  }

  /** Handles the path following logic when an end pose is specified */
  private void handlePathFollowing() {
    Pose2d currentPose = Drive.getInstance().getPose();
    double distanceToTarget = currentPose.getTranslation().getDistance(endPose.getTranslation());
    double rotationError = Math.abs(currentPose.getRotation().getRadians() - endPose.getRotation().getRadians());

    if (distanceToTarget > POSITION_TOLERANCE
        || rotationError > Math.toRadians(ROTATION_TOLERANCE)) {
      Logger.recordOutput("RealOutputs/x_error", xController.getPositionError());
      Logger.recordOutput("RealOutputs/y_error", yController.getPositionError());
      Logger.recordOutput("RealOutputs/theta_error", thetaController.getPositionError());
      // Move to target position
      ChassisSpeeds speeds = new ChassisSpeeds(
          LinearVelocity.ofBaseUnits(
              xController.calculate(currentPose.getX(), endPose.getX()), MetersPerSecond),
          LinearVelocity.ofBaseUnits(
              yController.calculate(currentPose.getY(), endPose.getY()), MetersPerSecond),
          AngularVelocity.ofBaseUnits(
              thetaController.calculate(
                  currentPose.getRotation().getRadians(), endPose.getRotation().getRadians()),
              RadiansPerSecond));
      Drive.getInstance()
          .runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds, Drive.getInstance().getPose().getRotation()));
    } else {
      handleScoring();
    }
  }

  /** Handles the scoring logic once in position */
  private void handleScoring() {
    double currentHeight = Elevator.getInstance().getHeight();
    if (Math.abs(targetHeight - currentHeight) < ELEVATOR_TOLERANCE
        || currentHeight > targetHeight) {
      if (tickCounter >= SCORING_DELAY_TICKS) {
        CoralIntake.getInstance()
            .setVelocity(LinearVelocity.ofBaseUnits(SCORING_VELOCITY, MetersPerSecond));
      } else {
        tickCounter++;
      }
    } else {
      CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Reset robot state
    // RobotContainer.coralIntakeCommand.setVelocity(
    // LinearVelocity.ofBaseUnits(DEFAULT_VELOCITY, MetersPerSecond));
    Elevator.getInstance().goToHeight(0);
    Drive.getInstance().stop();
    Drive.getInstance().setSdMultiplier(1);

    // Log any errors
    if (hasElevatorError) {
      System.err.println("AutoScoreCommand ended due to elevator error");
    }
    if (timeoutTimer.get() > COMMAND_TIMEOUT) {
      System.err.println("AutoScoreCommand ended due to timeout");
    }
  }

  @Override
  public boolean isFinished() {
    return !CoralIntake.getInstance().hasCoral()
        || hasElevatorError
        || timeoutTimer.get() > COMMAND_TIMEOUT;
  }
}
