package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ScoreCommand.ScoringLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.util.LoggingUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Command to automatically score game pieces at specified heights. This command handles both the
 * elevator movement and drive positioning. It includes safety checks, timeouts, and error handling.
 */
public class AutoScoreCommand extends Command {
  // Constants for position and timing - relaxed tolerances for smoother control
  public static double POSITION_TOLERANCE = 0.02; // meters (increased from 0.02)
  public static double ROTATION_TOLERANCE = 1.0; // degrees (increased from 0.6)
  private static final double ELEVATOR_TOLERANCE = 0.8; // meters
  private static final double SCORING_DELAY_TICKS = 3;
  private static final double SCORING_VELOCITY = 52.0; // meters per second
  // private static final double DEFAULT_VELOCITY = 6.0; // meters per second
  private static final double COMMAND_TIMEOUT = 10.0; // seconds

  // PID controller constants - reduced gains to prevent oscillation
  public static double X_PID_P = 13.0; // Reduced from 18.7
  public static double X_PID_I = 0.0; // Added I term
  public static double X_PID_D = 0.2; // Increased from 0.05
  public static double Y_PID_P = 13.0; // Reduced from 18.7
  public static double Y_PID_I = 0.0; // Added I term
  public static double Y_PID_D = 0.2; // Increased from 0.05
  public static double THETA_PID_P = 7.0; // Reduced from 9.9
  public static double THETA_PID_I = 0.0; // Added I term
  public static double THETA_PID_D = 0.15; // Increased from 0.1

  // Motion profile constants - smoother trajectory
  private static final double MAX_VELOCITY = 3; // Reduced from 2.0
  private static final double MAX_ACCELERATION = 2.5; // Reduced from 2.75
  private static final double MAX_ANGULAR_VELOCITY = 5.5; // Reduced from 2.0
  private static final double MAX_ANGULAR_ACCELERATION = 5.0; // Reduced from 2.75

  // Feedforward constants
  private static final double X_FEEDFORWARD_KS = 0.1; // Static friction
  private static final double X_FEEDFORWARD_KV = 0.05; // Velocity feedforward
  private static final double Y_FEEDFORWARD_KS = 0.1; // Static friction
  private static final double Y_FEEDFORWARD_KV = 0.05; // Velocity feedforward
  private static final double THETA_FEEDFORWARD_KS = 0.05; // Static friction
  private static final double THETA_FEEDFORWARD_KV = 0.02; // Velocity feedforward

  // Deadband constants to prevent tiny oscillations
  private static final double VELOCITY_DEADBAND = 0.01; // m/s
  private static final double ANGULAR_VELOCITY_DEADBAND = 0.02; // rad/s

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

    // Initialize controllers with constants including I term
    this.xController =
        new ProfiledPIDController(
            X_PID_P, X_PID_I, X_PID_D, new Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    this.yController =
        new ProfiledPIDController(
            Y_PID_P, Y_PID_I, Y_PID_D, new Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    this.thetaController =
        new ProfiledPIDController(
            THETA_PID_P,
            THETA_PID_I,
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
   * @param path The path to follow to the scoring position
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

    // Initialize controllers with constants including I term
    this.xController =
        new ProfiledPIDController(
            X_PID_P, X_PID_I, X_PID_D, new Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    this.yController =
        new ProfiledPIDController(
            Y_PID_P, Y_PID_I, Y_PID_D, new Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    this.thetaController =
        new ProfiledPIDController(
            THETA_PID_P,
            THETA_PID_I,
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
      this.endPose =
          new Pose2d(
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
        targetHeight = 21.6;
        break;
      case L3:
        targetHeight = 33.78;
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
    Pose2d currentPose = Drive.getInstance().getPose();

    // Reset PID controllers with current position and zero velocity
    xController.reset(currentPose.getX(), 0.0);
    yController.reset(currentPose.getY(), 0.0);
    thetaController.reset(currentPose.getRotation().getRadians(), 0.0);

    // Update PID constants from SmartDashboard if changed
    updatePIDFromDashboard();

    Drive.getInstance().setSdMultiplier(8.0);
    // Reset command state
    tickCounter = 0;
    hasElevatorError = false;
    timeoutTimer.reset();
    timeoutTimer.start();

    // Set initial positions and velocities
    setTargetHeight(level);
    CoralIntake.getInstance().setVelocity(LinearVelocity.ofBaseUnits(0, MetersPerSecond));
    // If the coral is not fully in the robot, prevent sending the elevator up so
    // nothing breaks
    if (CoralIntake.getInstance().getSensor3()) {
      Elevator.getInstance().goToHeight(targetHeight);
    }

    Logger.recordOutput("AutoScore/StartPose", currentPose);
    Logger.recordOutput("AutoScore/TargetPose", endPose);
  }

  /** Updates PID constants from SmartDashboard for live tuning */
  private void updatePIDFromDashboard() {
    // Update X PID using LoggingUtil for consistency
    double newXP = LoggingUtil.getTunableDouble("AutoScore/X_PID_P", X_PID_P);
    double newXI = LoggingUtil.getTunableDouble("AutoScore/X_PID_I", X_PID_I);
    double newXD = LoggingUtil.getTunableDouble("AutoScore/X_PID_D", X_PID_D);

    if (newXP != X_PID_P || newXI != X_PID_I || newXD != X_PID_D) {
      xController.setPID(newXP, newXI, newXD);
      X_PID_P = newXP;
      X_PID_I = newXI;
      X_PID_D = newXD;
    }

    // Update Y PID using LoggingUtil for consistency
    double newYP = LoggingUtil.getTunableDouble("AutoScore/Y_PID_P", Y_PID_P);
    double newYI = LoggingUtil.getTunableDouble("AutoScore/Y_PID_I", Y_PID_I);
    double newYD = LoggingUtil.getTunableDouble("AutoScore/Y_PID_D", Y_PID_D);

    if (newYP != Y_PID_P || newYI != Y_PID_I || newYD != Y_PID_D) {
      yController.setPID(newYP, newYI, newYD);
      Y_PID_P = newYP;
      Y_PID_I = newYI;
      Y_PID_D = newYD;
    }

    // Update Theta PID using LoggingUtil for consistency
    double newThetaP = LoggingUtil.getTunableDouble("AutoScore/THETA_PID_P", THETA_PID_P);
    double newThetaI = LoggingUtil.getTunableDouble("AutoScore/THETA_PID_I", THETA_PID_I);
    double newThetaD = LoggingUtil.getTunableDouble("AutoScore/THETA_PID_D", THETA_PID_D);

    if (newThetaP != THETA_PID_P || newThetaI != THETA_PID_I || newThetaD != THETA_PID_D) {
      thetaController.setPID(newThetaP, newThetaI, newThetaD);
      THETA_PID_P = newThetaP;
      THETA_PID_I = newThetaI;
      THETA_PID_D = newThetaD;
    }

    // Put current values on dashboard
    SmartDashboard.putNumber("AutoScore/X_PID_P", X_PID_P);
    SmartDashboard.putNumber("AutoScore/X_PID_I", X_PID_I);
    SmartDashboard.putNumber("AutoScore/X_PID_D", X_PID_D);
    SmartDashboard.putNumber("AutoScore/Y_PID_P", Y_PID_P);
    SmartDashboard.putNumber("AutoScore/Y_PID_I", Y_PID_I);
    SmartDashboard.putNumber("AutoScore/Y_PID_D", Y_PID_D);
    SmartDashboard.putNumber("AutoScore/THETA_PID_P", THETA_PID_P);
    SmartDashboard.putNumber("AutoScore/THETA_PID_I", THETA_PID_I);
    SmartDashboard.putNumber("AutoScore/THETA_PID_D", THETA_PID_D);
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
    // If the coral is not fully in the robot, prevent sending the elevator up so
    // nothing breaks
    if (CoralIntake.getInstance().getSensor3()) {
      Elevator.getInstance().goToHeight(targetHeight);
    }

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
    double rotationError =
        Math.abs(currentPose.getRotation().getRadians() - endPose.getRotation().getRadians());

    if (distanceToTarget > POSITION_TOLERANCE
        || rotationError > Math.toRadians(ROTATION_TOLERANCE)) {
      Logger.recordOutput("RealOutputs/x_error", xController.getPositionError());
      Logger.recordOutput("RealOutputs/y_error", yController.getPositionError());
      Logger.recordOutput("RealOutputs/theta_error", thetaController.getPositionError());

      // Calculate PID output with feedforward
      double xPIDOutput = xController.calculate(currentPose.getX(), endPose.getX());
      double yPIDOutput = yController.calculate(currentPose.getY(), endPose.getY());
      double thetaPIDOutput =
          thetaController.calculate(
              currentPose.getRotation().getRadians(), endPose.getRotation().getRadians());

      // Add feedforward terms for smoother control
      double xFeedforward =
          X_FEEDFORWARD_KS * Math.signum(xPIDOutput)
              + X_FEEDFORWARD_KV * xController.getSetpoint().velocity;
      double yFeedforward =
          Y_FEEDFORWARD_KS * Math.signum(yPIDOutput)
              + Y_FEEDFORWARD_KV * yController.getSetpoint().velocity;
      double thetaFeedforward =
          THETA_FEEDFORWARD_KS * Math.signum(thetaPIDOutput)
              + THETA_FEEDFORWARD_KV * thetaController.getSetpoint().velocity;

      // Combine PID and feedforward
      double xOutput = xPIDOutput + xFeedforward;
      double yOutput = yPIDOutput + yFeedforward;
      double thetaOutput = thetaPIDOutput + thetaFeedforward;

      // Apply deadband to prevent tiny oscillations
      xOutput = Math.abs(xOutput) > VELOCITY_DEADBAND ? xOutput : 0.0;
      yOutput = Math.abs(yOutput) > VELOCITY_DEADBAND ? yOutput : 0.0;
      thetaOutput = Math.abs(thetaOutput) > ANGULAR_VELOCITY_DEADBAND ? thetaOutput : 0.0;

      // Create chassis speeds with proper velocity values (not wrapped in units)
      ChassisSpeeds speeds = new ChassisSpeeds(xOutput, yOutput, thetaOutput);

      Logger.recordOutput("RealOutputs/x_velocity_command", xOutput);
      Logger.recordOutput("RealOutputs/y_velocity_command", yOutput);
      Logger.recordOutput("RealOutputs/theta_velocity_command", thetaOutput);

      // Apply field-relative transformation and send to drive using precision control
      Drive.getInstance()
          .runVelocityPrecision(
              ChassisSpeeds.fromFieldRelativeSpeeds(speeds, currentPose.getRotation()));
    } else {
      // Stop the drive when in position
      Drive.getInstance().stop();
      handleScoring();
    }
  }

  /** Handles the scoring logic once in position */
  private void handleScoring() {
    Drive.getInstance().stop();
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
