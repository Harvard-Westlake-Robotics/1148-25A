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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.CameraIO.TimestampedPose;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static Drive instance;

  public static Drive getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          instance = new Drive(
              new GyroIOPigeon2(),
              new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
              new ModuleIOTalonFXReal(TunerConstants.FrontRight),
              new ModuleIOTalonFXReal(TunerConstants.BackLeft),
              new ModuleIOTalonFXReal(TunerConstants.BackRight),
              pose -> {
              });
          break;
        case SIM:
          instance = new Drive(
              new GyroIOSim(RobotContainer.driveSimulation.getGyroSimulation()),
              new ModuleIOTalonFXSim(
                  TunerConstants.FrontLeft, RobotContainer.driveSimulation.getModules()[0]),
              new ModuleIOTalonFXSim(
                  TunerConstants.FrontRight, RobotContainer.driveSimulation.getModules()[1]),
              new ModuleIOTalonFXSim(
                  TunerConstants.BackLeft, RobotContainer.driveSimulation.getModules()[2]),
              new ModuleIOTalonFXSim(
                  TunerConstants.BackRight, RobotContainer.driveSimulation.getModules()[3]),
              RobotContainer.driveSimulation::setSimulationWorldPose);
          break;
        default:
          // Replayed robot, disable IO implementations
          instance = new Drive(
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
          break;
      }
    }
    return instance;
  }

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
      AlertType.kError);

  @AutoLogOutput
  private boolean visionActive = true;

  public boolean isVisionActive() {
    return visionActive;
  }

  public void setVisionActive(boolean visionActive) {
    this.visionActive = visionActive;
  }

  static final Lock odometryLock = new ReentrantLock();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Drive.getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
      lastModulePositions, new Pose2d());

  // Discretization time constant
  private static final double DISCRETIZATION_TIME_SECONDS = 0.02;

  private double sdMultiplier = 1.0;

  public void setSdMultiplier(double sdMultiplier) {
    this.sdMultiplier = sdMultiplier;
  }

  // Drift compensation variables
  private Translation2d driftCompensation = new Translation2d();
  private boolean isDriftCompensationEnabled = true;
  private double driftCompensationX = 0.0; // Forward/backward compensation
  private double driftCompensationY = 0.0; // Left/right compensation

  // Create and configure a drivetrain simulation configuration
  public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
      // Specify robot mass
      .withRobotMass(Kilograms.of(TunerConstants.ROBOT_MASS_KG)) // Set robot mass in kg
      // Specify gyro type (for realistic gyro drifting and error simulation)
      .withGyro(COTS.ofPigeon2())
      // Specify module positions
      .withCustomModuleTranslations(getModuleTranslations())
      // Specify swerve module (for realistic swerve dynamics)
      .withSwerveModule(
          new SwerveModuleSimulationConfig(
              DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
              DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
              TunerConstants.kDriveGearRatio, // Drive motor gear ratio.
              TunerConstants.kSteerGearRatio, // Steer motor gear ratio.
              TunerConstants.kDriveFrictionVoltage, // Drive friction voltage.
              TunerConstants.kSteerFrictionVoltage, // Steer friction voltage
              Inches.of(2.15), // Wheel radius
              TunerConstants.kSteerInertia, // Steer MOI
              1.2)) // Wheel COF
      // Configures the track length and track width (spacing between swerve modules)
      .withTrackLengthTrackWidth(Inches.of(21), Inches.of(21))
      // Configures the bumper size (dimensions of the robot bumper)
      .withBumperSize(Inches.of(33.6), Inches.of(33.6));

  private final Consumer<Pose2d> resetSimulationPoseCallback;

  // Field dimensions and vision constants
  private static final double FIELD_WIDTH_METERS = 16.54;
  private static final double FIELD_HEIGHT_METERS = 8.02;
  private static final double FIELD_BORDER_MARGIN_METERS = 0.00;

  public Drive(
      GyroIO gyroIO,
      ModuleIOTalonFX flModuleIO,
      ModuleIOTalonFX frModuleIO,
      ModuleIOTalonFX blModuleIO,
      ModuleIOTalonFX brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallback) {
    this.gyroIO = gyroIO;
    this.resetSimulationPoseCallback = resetSimulationPoseCallback;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(
                TunerConstants.PP_TRANSLATION_P,
                TunerConstants.PP_TRANSLATION_I,
                TunerConstants.PP_TRANSLATION_D),
            new PIDConstants(
                TunerConstants.PP_ROTATION_P,
                TunerConstants.PP_ROTATION_I,
                TunerConstants.PP_ROTATION_D)),
        TunerConstants.PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    // Set simulation pose
    setPose(new Pose2d());

    // Prevents double initialization since this constructor is called directly in
    // RobotContainer
    Drive.instance = this;

    // Setup NetworkTables communication
    NetworkCommunicator.getInstance().init();
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("RealOutputs/Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();
    Vision.getInstance().setRobotOrientation(getPose().getRotation());
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      Vision.getInstance().setUseMegaTag2(false);
    } else {
      Vision.getInstance().setUseMegaTag2(true);
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
            modulePositions[moduleIndex].distanceMeters
                - lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

      Logger.recordOutput("Odometry/Velocity/LinearVelocity", getLinearVelocity());
      Logger.recordOutput("Odometry/Velocity/AngularVelocity", getAngularVelocity());

      TimestampedPose[] timestampedPoses = Vision.getInstance().getTimestampedPoses();

      for (TimestampedPose pose : timestampedPoses) {
        if (pose != null && shouldAcceptPose(pose.pose)) {
          addVisionMeasurement(
              pose.pose,
              pose.timestamp,
              VecBuilder.fill(
                  sdMultiplier * Constants.Vision.xyStdDev,
                  sdMultiplier * Constants.Vision.xyStdDev,
                  sdMultiplier * Constants.Vision.rStdDev));
        }
      }

      // Update gyro alert
      gyroDisconnectedAlert.set(
          !gyroInputs.connected && Constants.currentMode != Constants.Mode.SIM);
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Chassis speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Convert to discrete time for better accuracy
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, DISCRETIZATION_TIME_SECONDS);

    // Calculate module setpoints
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);

    // Enforce velocity limits
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Apply optimization to prevent module flipping
    for (int i = 0; i < 4; i++) {
      setpointStates[i].optimize(modules[i].getAngle());
    }

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Applies drift compensation to chassis speeds. This corrects for systematic
   * drift caused by
   * wheel slippage, carpet effects, etc.
   *
   * @param speeds The original chassis speeds
   * @return Compensated chassis speeds
   */
  private ChassisSpeeds applyDriftCompensation(ChassisSpeeds speeds) {
    if (!isDriftCompensationEnabled) {
      return speeds;
    }

    // Apply compensation based on the robot's current velocity direction
    double compensatedVx = speeds.vxMetersPerSecond + (driftCompensationX * Math.abs(speeds.vxMetersPerSecond));
    double compensatedVy = speeds.vyMetersPerSecond + (driftCompensationY * Math.abs(speeds.vyMetersPerSecond));

    return new ChassisSpeeds(compensatedVx, compensatedVy, speeds.omegaRadiansPerSecond);
  }

  /**
   * Updates drift compensation factors based on observed vs expected motion.
   *
   * @param expectedTranslation The expected robot translation
   * @param actualTranslation   The actual robot translation
   */
  public void updateDriftCompensation(
      Translation2d expectedTranslation, Translation2d actualTranslation) {
    if (!isDriftCompensationEnabled) {
      return;
    }

    // Calculate the error between expected and actual movement
    Translation2d error = actualTranslation.minus(expectedTranslation);

    // Only update if we moved a significant distance
    double distanceMoved = expectedTranslation.getNorm();
    if (distanceMoved < 0.1) {
      return;
    }

    // Calculate compensation factors (error per unit distance moved)
    double errorX = error.getX() / distanceMoved;
    double errorY = error.getY() / distanceMoved;

    // Apply learning rate and clamp to maximum compensation
    driftCompensationX += DRIFT_LEARNING_RATE * errorX;
    driftCompensationY += DRIFT_LEARNING_RATE * errorY;

    driftCompensationX = MathUtil.clamp(driftCompensationX, -DRIFT_COMPENSATION_MAX, DRIFT_COMPENSATION_MAX);
    driftCompensationY = MathUtil.clamp(driftCompensationY, -DRIFT_COMPENSATION_MAX, DRIFT_COMPENSATION_MAX);

    // Log the compensation factors for tuning
    Logger.recordOutput("Drive/DriftCompensation/X", driftCompensationX);
    Logger.recordOutput("Drive/DriftCompensation/Y", driftCompensationY);
    Logger.recordOutput("Drive/DriftCompensation/ErrorX", errorX);
    Logger.recordOutput("Drive/DriftCompensation/ErrorY", errorY);
  }

  /**
   * Enables or disables drift compensation.
   *
   * @param enabled Whether drift compensation should be enabled
   */
  public void setDriftCompensationEnabled(boolean enabled) {
    this.isDriftCompensationEnabled = enabled;
  }

  /**
   * Gets the current drift compensation factors.
   *
   * @return Translation2d containing X and Y compensation factors
   */
  public Translation2d getDriftCompensation() {
    return new Translation2d(driftCompensationX, driftCompensationY);
  }

  /** Resets drift compensation factors to zero. */
  public void resetDriftCompensation() {
    driftCompensationX = 0.0;
    driftCompensationY = 0.0;
  }

  /** Starts drift calibration mode. */
  public void startDriftCalibration() {
    isDriftCalibrationActive = true;
    driftCalibrationStartPose = getPose();
    driftCalibrationStartTime = Timer.getFPGATimestamp();
    Logger.recordOutput("Drive/DriftCalibration/Active", true);
  }

  /** Stops drift calibration mode and processes results. */
  public void stopDriftCalibration() {
    if (!isDriftCalibrationActive) {
      return;
    }

    isDriftCalibrationActive = false;

    // Calculate actual vs expected movement
    Pose2d currentPose = getPose();
    Translation2d actualMovement = currentPose.getTranslation().minus(driftCalibrationStartPose.getTranslation());

    if (driftCalibrationExpectedPosition.getNorm() > 0.1) {
      updateDriftCompensation(driftCalibrationExpectedPosition, actualMovement);
    }

    Logger.recordOutput("Drive/DriftCalibration/Active", false);
    Logger.recordOutput(
        "Drive/DriftCalibration/ExpectedMovement", driftCalibrationExpectedPosition);
    Logger.recordOutput("Drive/DriftCalibration/ActualMovement", actualMovement);
  }

  /**
   * Sets the expected position for drift calibration.
   *
   * @param expectedPosition The expected robot position relative to start
   */
  public void setDriftCalibrationExpectedPosition(Translation2d expectedPosition) {
    this.driftCalibrationExpectedPosition = expectedPosition;
  }

  /**
   * Checks if drift calibration is currently active.
   *
   * @return True if drift calibration is active
   */
  public boolean isDriftCalibrationActive() {
    return isDriftCalibrationActive;
  }

  /**
   * Runs the drive with precision velocity control for auto scoring. This reduces
   * jitter by using
   * smoother control during position commands.
   *
   * @param speeds Chassis speeds in meters/sec
   */
  public void runVelocityPrecision(ChassisSpeeds speeds) {
    // Apply drift compensation with reduced sensitivity
    if (isDriftCompensationEnabled) {
      double compensatedVx = speeds.vxMetersPerSecond
          + (driftCompensationX * Math.abs(speeds.vxMetersPerSecond) * 0.5); // Reduced by 50%
      double compensatedVy = speeds.vyMetersPerSecond
          + (driftCompensationY * Math.abs(speeds.vyMetersPerSecond) * 0.5); // Reduced by 50%
      speeds = new ChassisSpeeds(compensatedVx, compensatedVy, speeds.omegaRadiansPerSecond);
    }

    // Run normal velocity control with optimized processing order
    runVelocity(speeds);
  }

  /**
   * Runs the drive in a straight line with the specified drive output. Used for
   * system
   * identification.
   *
   * @param output Voltage output to apply to all modules
   */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive by setting zero chassis speeds. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement.
   * The modules will
   * return to their normal orientations the next time a nonzero velocity is
   * requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all modules.
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Returns the module positions (turn angles and drive positions) for all
   * modules.
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec. */
  public double getFFCharacterizationVelocity() {
    double totalVelocity = 0.0;
    for (int i = 0; i < 4; i++) {
      totalVelocity += modules[i].getFFCharacterizationVelocity();
    }
    return totalVelocity / 4.0;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    resetSimulationPoseCallback.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / TunerConstants.DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  private double getLinearVelocity() {
    ChassisSpeeds speeds = getChassisSpeeds();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    return linearSpeed;
  }

  private double getAngularVelocity() {
    double angularVelocity = gyroInputs.yawVelocityRadPerSec;
    return Units.degreesToRadians(angularVelocity);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /**
   * Determines if a vision pose should be accepted based on various criteria.
   *
   * @param pose The vision pose
   * @return True if the pose should be accepted, false otherwise
   */
  public boolean shouldAcceptPose(Pose2d pose) {
    // Always accept poses when disabled
    if (DriverStation.isDisabled()) {
      return true;
    }

    // Reject if cameras are off
    if (!visionActive) {
      return false;
    }

    // Reject if rotating too quickly
    if (Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec) >= Constants.Vision.MAX_YAW_RATE_DEGREES_PER_SEC) {
      return false;
    }

    // Reject if outside field bounds
    if (isOutsideFieldBounds(pose)) {
      return false;
    }

    // Accept the pose
    return true;
  }

  /** Checks if a pose is outside the field boundaries. */
  private boolean isOutsideFieldBounds(Pose2d pose) {
    return pose.getX() < -FIELD_BORDER_MARGIN_METERS
        || pose.getX() > FIELD_WIDTH_METERS + FIELD_BORDER_MARGIN_METERS
        || pose.getY() < -FIELD_BORDER_MARGIN_METERS
        || pose.getY() > FIELD_HEIGHT_METERS + FIELD_BORDER_MARGIN_METERS;
  }
}
