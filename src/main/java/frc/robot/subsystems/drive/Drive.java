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

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
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
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Camera.BaseCam.AprilTagResult;
import frc.robot.Camera.LimeLightCam;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.LoggingUtil;
import frc.robot.util.PerformanceMonitor;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // Force Logger import to stay - DO NOT REMOVE
  @SuppressWarnings("unused")
  private static final Class<?> LOGGER_CLASS = org.littletonrobotics.junction.Logger.class;

  private static Drive instance;

  public static Drive getInstance() {
    return instance;
  }

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD()
      ? 250.0
      : 100.0;
  public static final double DRIVE_BASE_RADIUS = Math.max(
      Math.max(
          Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
          Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
      Math.max(
          Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
          Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  public static final double ROBOT_MASS_KG = 54.088;
  public static final double ROBOT_MOI = 6.883;
  public static final double WHEEL_COF = 1.2;
  public static final RobotConfig PP_CONFIG = new RobotConfig(
      ROBOT_MASS_KG,
      ROBOT_MOI,
      new ModuleConfig(
          TunerConstants.FrontLeft.WheelRadius,
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          WHEEL_COF,
          DCMotor.getKrakenX60Foc(1)
              .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
          TunerConstants.FrontLeft.SlipCurrent,
          1),
      getModuleTranslations());
  public static final PathConstraints PP_CONSTRAINTS = new PathConstraints(
      LinearVelocity.ofBaseUnits(7.5, MetersPerSecond),
      LinearAcceleration.ofBaseUnits(5.5, MetersPerSecondPerSecond),
      AngularVelocity.ofBaseUnits(1020, DegreesPerSecond),
      AngularAcceleration.ofBaseUnits(
          2400, DegreesPerSecondPerSecond)); // PathConstraints.unlimitedConstraints(12);
  // new PathConstraints(
  // TunerConstants.kSpeedAt12Volts,
  // LinearAcceleration.ofBaseUnits(5.0, MetersPerSecondPerSecond),
  // AngularVelocity.ofBaseUnits(755, DegreesPerSecond),
  // AngularAcceleration.ofBaseUnits(1054, DegreesPerSecondPerSecond));
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
      AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
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

  private final LimeLightCam limelight_a = new LimeLightCam("limelight-a", false);
  private final LimeLightCam limelight_b = new LimeLightCam("limelight-b", false);
  private final LimeLightCam limelight_c = new LimeLightCam("limelight-c", false);

  private final LimeLightCam[] limelights = new LimeLightCam[] { limelight_a, limelight_b, limelight_c };

  private double PP_ROTATION_P = 5.05;
  private double PP_ROTATION_I = 0.00;
  private double PP_ROTATION_D = 0.00;
  private double PP_TRANSLATION_P = 4.45;
  private double PP_TRANSLATION_I = 0.00;
  private double PP_TRANSLATION_D = 0.0;

  private boolean constantsChangedThisTick = false;
  private boolean limeLightsActive = true;

  // Drift compensation variables
  private Translation2d driftCompensation = new Translation2d();
  private boolean isDriftCompensationEnabled = true;
  private double driftCompensationX = 0.0; // Forward/backward compensation
  private double driftCompensationY = 0.0; // Left/right compensation

  // Drift calibration variables
  private boolean isDriftCalibrationActive = false;
  private Pose2d driftCalibrationStartPose = new Pose2d();
  private Translation2d driftCalibrationExpectedPosition = new Translation2d();
  private double driftCalibrationStartTime = 0.0;

  // Traction control variables
  private double[] filteredWheelSpeeds = new double[4]; // Smoothed wheel speeds
  private double[] previousWheelSpeeds = new double[4]; // For acceleration calculation
  private double lastTractionControlUpdate = 0.0;
  private boolean[] moduleSlipping = new boolean[4]; // Per-module slip detection
  private double skiddingRatio = 1.0; // Current skidding ratio
  private double lastChassisSpeed = 0.0; // For acceleration limiting
  private double[] tractionControlMultipliers = { 1.0, 1.0, 1.0, 1.0 }; // Per-module power reduction

  public boolean isLimeLightsActive() {
    return limeLightsActive;
  }

  public void setLimeLightsActive(boolean limeLightsActive) {
    this.limeLightsActive = limeLightsActive;
  }

  private double sdMultiplier = 1;

  public double getSdMultiplier() {
    return sdMultiplier;
  }

  public void setSdMultiplier(double sdMultiplier) {
    this.sdMultiplier = sdMultiplier;
  }

  private double xyStdDevCoeff = 6.85;
  private double rStdDevCoeff = 6.85;
  private double xyStdDev = 0.8;
  private double rStdDev = 6.2;

  // Discretization time constant
  private static final double DISCRETIZATION_TIME_SECONDS = 0.02;

  // Field dimensions and vision constants
  private static final double FIELD_WIDTH_METERS = 16.54;
  private static final double FIELD_HEIGHT_METERS = 8.02;
  private static final double FIELD_BORDER_MARGIN_METERS = 0.05;
  private static final double MAX_YAW_RATE_DEGREES_PER_SEC = 520.0;

  // Enhanced vision filtering constants (tunable via SmartDashboard)
  private static double VISION_AMBIGUITY_THRESHOLD = 0.3; // Maximum acceptable ambiguity
  private static double VISION_MAX_DISTANCE_METERS = 6.0; // Maximum detection distance
  private static double VISION_MAX_POSE_DIFFERENCE_METERS = 2.0; // Maximum pose jump allowed

  // Drift compensation constants
  private static final double DRIFT_CALIBRATION_VELOCITY = 1.0; // m/s for calibration
  private static final double DRIFT_CALIBRATION_DISTANCE = 3.0; // meters to travel during calibration
  private static final double DRIFT_COMPENSATION_MAX = 0.15; // Maximum compensation factor
  private static final double DRIFT_LEARNING_RATE = 0.1; // How quickly to adapt compensation

  // Reef positioning constantsg
  private static final double ROBOT_REEF_OFFSET_METERS = -0.3556;
  private static final double BLUE_REEF_CENTER_X = 4.5;
  private static final double RED_REEF_CENTER_X = 13.05;
  private static final double REEF_CENTER_Y = 4.025;
  private static final double REEF_CENTER_RADIUS = 1.1721;
  private static final double ELEVATOR_ANGLE_DEGREES = 40.0;

  // Create and configure a drivetrain simulation configuration
  public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
      // Specify robot mass
      .withRobotMass(Kilograms.of(ROBOT_MASS_KG)) // Set robot mass in kg
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

  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  // Car-like drift mode constants
  private static final double DRIFT_FRONT_MAX_STEERING_DEGREES = 70.0; // Maximum front wheel steering angle
  private static final double DRIFT_REAR_TOE_IN_DEGREES = 2.0; // Slight toe-in for rear wheels
  private static final double DRIFT_REAR_POWER_MULTIPLIER = 1.5; // High power to rear wheels for oversteer
  private static final double DRIFT_FRONT_POWER_MULTIPLIER = 0.0; // No power to front wheels (coast)
  private static final double DRIFT_STEERING_SENSITIVITY = 0.8; // How responsive steering is

  // Traction Control Constants - Team 1690 inspired skid detection
  public static double SKID_DETECTION_THRESHOLD = 1.2; // Ratio threshold for detecting slip
  public static double TRACTION_CONTROL_REDUCTION = 0.7; // Reduce power to 70% when slip detected
  public static double SKID_DETECTION_MIN_SPEED = 0.5; // Minimum speed to check for skid (m/s)
  public static double ACCELERATION_LIMIT_THRESHOLD = 3.0; // Max acceleration before limiting (m/s²)
  public static boolean TRACTION_CONTROL_ENABLED = false; // Default disabled for testing
  private static final double SKID_DETECTION_UPDATE_RATE = 0.02; // 50Hz update rate
  private static final double VELOCITY_FILTER_ALPHA = 0.8; // Low-pass filter for velocity smoothing
  private static final double CHASSIS_ROTATION_COMPENSATION_THRESHOLD = 0.1; // rad/s

  public Drive(
      GyroIO gyroIO,
      ModuleIOTalonFX flModuleIO,
      ModuleIOTalonFX frModuleIO,
      ModuleIOTalonFX blModuleIO,
      ModuleIOTalonFX brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack) {
    this.gyroIO = gyroIO;
    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template (commented out due to missing imports)
    // HAL.report(tResourceType.kResourceType_RobotDrive,
    // tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(PP_TRANSLATION_P, PP_TRANSLATION_I, PP_TRANSLATION_D),
            new PIDConstants(PP_ROTATION_P, PP_ROTATION_I, PP_ROTATION_D)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          org.littletonrobotics.junction.Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          org.littletonrobotics.junction.Logger.recordOutput(
              "Odometry/TrajectorySetpoint", targetPose);
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
    setPose(new Pose2d());

    Drive.instance = this;

    // Setup NetworkTables communication
    NetworkCommunicator.getInstance().init();
  }

  @Override
  public void periodic() {
    // Start performance monitoring
    PerformanceMonitor monitor = PerformanceMonitor.getInstance();
    double periodicStartTime = monitor.startTiming("Drive");

    // Core odometry and sensor updates (always execute)
    double odometryStartTime = monitor.startTiming("Drive_Odometry");
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Update odometry with improved structure and logging
    updateOdometry();
    monitor.endTiming("Drive_Odometry", odometryStartTime);

    // Vision updates (adaptive frequency based on performance)
    if (shouldUpdateVision()) {
      double visionStartTime = monitor.startTiming("Drive_Vision");
      updateVisionMeasurements();
      updateVisionSystems();
      monitor.endTiming("Drive_Vision", visionStartTime);
    }

    // Critical safety updates (always execute)
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      // Log empty setpoint states when disabled
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Non-critical updates (skip if performance is poor)
    if (!monitor.shouldSkipNonCriticalOperations("Drive")) {
      double loggingStartTime = monitor.startTiming("Drive_Logging");
      logDriveSystemState();
      monitor.endTiming("Drive_Logging", loggingStartTime);

      double dashboardStartTime = monitor.startTiming("Drive_Dashboard");
      updateDashboardValues();
      monitor.endTiming("Drive_Dashboard", dashboardStartTime);
    } else {
      // In performance-critical mode, only update essential dashboard values
      updateCriticalDashboardValues();
    }

    // Update gyro alert (critical for safety)
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    // End performance monitoring
    monitor.endTiming("Drive", periodicStartTime);
  }

  /** Updates dashboard values for tuning. */
  private void updateDashboardValues() {
    updatePIDValues();
    updateVisionParameters();
    // updatePathPlannerIfNeeded();
    updateScoringParameters();
  }

  /** Updates PID values from the dashboard. */
  private void updatePIDValues() {
    // Update translation PID constants
    PP_TRANSLATION_P = SmartDashboard.getNumber("PP_TRANSLATION_P", PP_TRANSLATION_P);
    SmartDashboard.putNumber("PP_TRANSLATION_P", PP_TRANSLATION_P);

    PP_TRANSLATION_I = SmartDashboard.getNumber("PP_TRANSLATION_I", PP_TRANSLATION_I);
    SmartDashboard.putNumber("PP_TRANSLATION_I", PP_TRANSLATION_I);

    PP_TRANSLATION_D = SmartDashboard.getNumber("PP_TRANSLATION_D", PP_TRANSLATION_D);
    SmartDashboard.putNumber("PP_TRANSLATION_D", PP_TRANSLATION_D);

    // Update rotation PID constants
    PP_ROTATION_P = SmartDashboard.getNumber("PP_ROTATION_P", PP_ROTATION_P);
    SmartDashboard.putNumber("PP_ROTATION_P", PP_ROTATION_P);

    PP_ROTATION_I = SmartDashboard.getNumber("PP_ROTATION_I", PP_ROTATION_I);
    SmartDashboard.putNumber("PP_ROTATION_I", PP_ROTATION_I);

    PP_ROTATION_D = SmartDashboard.getNumber("PP_ROTATION_D", PP_ROTATION_D);
    SmartDashboard.putNumber("PP_ROTATION_D", PP_ROTATION_D);

    // Mark that constants have changed
    constantsChangedThisTick = true;
  }

  /** Updates vision parameters from the dashboard. */
  private void updateVisionParameters() {
    xyStdDevCoeff = SmartDashboard.getNumber("xyStdDevCoeff", xyStdDevCoeff);
    SmartDashboard.putNumber("xyStdDevCoeff", xyStdDevCoeff);

    rStdDevCoeff = SmartDashboard.getNumber("rStdDevCoeff", rStdDevCoeff);
    SmartDashboard.putNumber("rStdDevCoeff", rStdDevCoeff);
  }

  private void updateScoringParameters() {
    AutoScoreCommand.X_PID_P = SmartDashboard.getNumber("X_PID_P", AutoScoreCommand.X_PID_P);
    SmartDashboard.putNumber("X_PID_P", AutoScoreCommand.X_PID_P);
    AutoScoreCommand.X_PID_D = SmartDashboard.getNumber("X_PID_D", AutoScoreCommand.X_PID_D);
    SmartDashboard.putNumber("X_PID_D", AutoScoreCommand.X_PID_D);

    AutoScoreCommand.Y_PID_P = SmartDashboard.getNumber("Y_PID_P", AutoScoreCommand.Y_PID_P);
    SmartDashboard.putNumber("Y_PID_P", AutoScoreCommand.Y_PID_P);
    AutoScoreCommand.Y_PID_D = SmartDashboard.getNumber("Y_PID_D", AutoScoreCommand.Y_PID_D);
    SmartDashboard.putNumber("Y_PID_D", AutoScoreCommand.Y_PID_D);

    AutoScoreCommand.THETA_PID_P = SmartDashboard.getNumber("THETA_PID_P", AutoScoreCommand.THETA_PID_P);
    SmartDashboard.putNumber("THETA_PID_P", AutoScoreCommand.THETA_PID_P);
    AutoScoreCommand.THETA_PID_D = SmartDashboard.getNumber("THETA_PID_D", AutoScoreCommand.THETA_PID_D);
    SmartDashboard.putNumber("THETA_PID_D", AutoScoreCommand.THETA_PID_D);

    AutoScoreCommand.POSITION_TOLERANCE = SmartDashboard.getNumber("POSITION_TOLERANCE",
        AutoScoreCommand.POSITION_TOLERANCE);
    SmartDashboard.putNumber("POSITION_TOLERANCE", AutoScoreCommand.POSITION_TOLERANCE);
    AutoScoreCommand.ROTATION_TOLERANCE = SmartDashboard.getNumber("ROTATION_TOLERANCE",
        AutoScoreCommand.ROTATION_TOLERANCE);
    SmartDashboard.putNumber("ROTATION_TOLERANCE", AutoScoreCommand.ROTATION_TOLERANCE);

    // Update traction control system
    updateTractionControl();

    // Update tunable traction control constants from SmartDashboard
    SKID_DETECTION_THRESHOLD = SmartDashboard.getNumber("TractionControl/SkidThreshold", SKID_DETECTION_THRESHOLD);
    SmartDashboard.putNumber("TractionControl/SkidThreshold", SKID_DETECTION_THRESHOLD);

    TRACTION_CONTROL_REDUCTION = SmartDashboard.getNumber("TractionControl/PowerReduction", TRACTION_CONTROL_REDUCTION);
    SmartDashboard.putNumber("TractionControl/PowerReduction", TRACTION_CONTROL_REDUCTION);

    SKID_DETECTION_MIN_SPEED = SmartDashboard.getNumber("TractionControl/MinSpeed", SKID_DETECTION_MIN_SPEED);
    SmartDashboard.putNumber("TractionControl/MinSpeed", SKID_DETECTION_MIN_SPEED);

    ACCELERATION_LIMIT_THRESHOLD = SmartDashboard.getNumber("TractionControl/AccelLimit", ACCELERATION_LIMIT_THRESHOLD);
    SmartDashboard.putNumber("TractionControl/AccelLimit", ACCELERATION_LIMIT_THRESHOLD);

    TRACTION_CONTROL_ENABLED = SmartDashboard.getBoolean("TractionControl/Enabled", TRACTION_CONTROL_ENABLED);
    SmartDashboard.putBoolean("TractionControl/Enabled", TRACTION_CONTROL_ENABLED);

    // Update all tunable parameters from dashboard
    updateTunableParameters();
  }

  /** Updates PathPlanner if PID constants have changed. */
  private void updatePathPlannerIfNeeded() {
    if (!constantsChangedThisTick) {
      return;
    }

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(PP_TRANSLATION_P, PP_TRANSLATION_I, PP_TRANSLATION_D),
            new PIDConstants(PP_ROTATION_P, PP_ROTATION_I, PP_ROTATION_D)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    constantsChangedThisTick = false;
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
   * Updates traction control system based on Team 1690's skid detection
   * algorithm. Calculates
   * skidding ratio and applies power reduction to slipping wheels.
   */
  private void updateTractionControl() {
    if (!TRACTION_CONTROL_ENABLED) {
      // Reset all multipliers to 1.0 when disabled
      for (int i = 0; i < 4; i++) {
        tractionControlMultipliers[i] = 1.0;
        moduleSlipping[i] = false;
      }
      skiddingRatio = 1.0;
      return;
    }

    double currentTime = Timer.getFPGATimestamp();

    // Only update at specified rate to avoid noise
    if (currentTime - lastTractionControlUpdate < SKID_DETECTION_UPDATE_RATE) {
      return;
    }
    lastTractionControlUpdate = currentTime;

    // Get current module states
    SwerveModuleState[] currentStates = getModuleStates();
    ChassisSpeeds currentChassisSpeeds = getChassisSpeeds();

    // Filter wheel speeds to reduce noise
    for (int i = 0; i < 4; i++) {
      double rawSpeed = Math.abs(currentStates[i].speedMetersPerSecond);
      filteredWheelSpeeds[i] = VELOCITY_FILTER_ALPHA * filteredWheelSpeeds[i]
          + (1.0 - VELOCITY_FILTER_ALPHA) * rawSpeed;
    }

    // Calculate chassis translational speed
    double chassisSpeed = Math.hypot(currentChassisSpeeds.vxMetersPerSecond, currentChassisSpeeds.vyMetersPerSecond);

    // Only perform skid detection above minimum speed threshold
    if (chassisSpeed < SKID_DETECTION_MIN_SPEED) {
      for (int i = 0; i < 4; i++) {
        moduleSlipping[i] = false;
        tractionControlMultipliers[i] = 1.0;
      }
      skiddingRatio = 1.0;
      return;
    }

    // Calculate expected wheel speeds based on chassis motion
    double[] expectedWheelSpeeds = calculateExpectedWheelSpeeds(currentChassisSpeeds);

    // Find min and max wheel speeds after compensating for chassis rotation
    double minSpeed = Double.MAX_VALUE;
    double maxSpeed = 0.0;

    for (int i = 0; i < 4; i++) {
      // Compensate for chassis rotation component
      double compensatedSpeed = filteredWheelSpeeds[i];
      if (Math.abs(currentChassisSpeeds.omegaRadiansPerSecond) > CHASSIS_ROTATION_COMPENSATION_THRESHOLD) {
        // Subtract expected rotation contribution
        compensatedSpeed = Math.max(
            0,
            compensatedSpeed
                - Math.abs(
                    getModuleTranslations()[i].getNorm()
                        * currentChassisSpeeds.omegaRadiansPerSecond));
      }

      minSpeed = Math.min(minSpeed, compensatedSpeed);
      maxSpeed = Math.max(maxSpeed, compensatedSpeed);
    }

    // Calculate skidding ratio (Team 1690 algorithm)
    skiddingRatio = minSpeed > 0.001 ? maxSpeed / minSpeed : 1.0;

    // Detect slip and apply traction control
    if (skiddingRatio > SKID_DETECTION_THRESHOLD) {
      // Apply traction control to modules that are slipping
      for (int i = 0; i < 4; i++) {
        // Check if this module is significantly faster than expected
        double speedError = filteredWheelSpeeds[i] - expectedWheelSpeeds[i];
        boolean isSlipping = speedError > (chassisSpeed * 0.2); // 20% speed error threshold

        moduleSlipping[i] = isSlipping;

        if (isSlipping) {
          // Reduce power to slipping module
          tractionControlMultipliers[i] = TRACTION_CONTROL_REDUCTION;
          Logger.recordOutput("TractionControl/Module" + i + "Slipping", true);
        } else {
          // Gradually restore power to non-slipping modules
          tractionControlMultipliers[i] = Math.min(1.0, tractionControlMultipliers[i] + 0.05);
          Logger.recordOutput("TractionControl/Module" + i + "Slipping", false);
        }
      }
    } else {
      // No slip detected, gradually restore all modules to full power
      for (int i = 0; i < 4; i++) {
        moduleSlipping[i] = false;
        tractionControlMultipliers[i] = Math.min(1.0, tractionControlMultipliers[i] + 0.02);
        Logger.recordOutput("TractionControl/Module" + i + "Slipping", false);
      }
    }

    // Acceleration limiting
    double acceleration = (chassisSpeed - lastChassisSpeed) / SKID_DETECTION_UPDATE_RATE;
    if (Math.abs(acceleration) > ACCELERATION_LIMIT_THRESHOLD) {
      // Apply additional reduction to all modules during aggressive acceleration
      for (int i = 0; i < 4; i++) {
        tractionControlMultipliers[i] *= 0.9;
      }
    }

    lastChassisSpeed = chassisSpeed;

    // Log telemetry
    Logger.recordOutput("TractionControl/SkiddingRatio", skiddingRatio);
    Logger.recordOutput("TractionControl/ChassisSpeed", chassisSpeed);
    Logger.recordOutput("TractionControl/SlipDetected", skiddingRatio > SKID_DETECTION_THRESHOLD);
    Logger.recordOutput("TractionControl/TractionMultipliers", tractionControlMultipliers);
    Logger.recordOutput("TractionControl/FilteredWheelSpeeds", filteredWheelSpeeds);
  }

  /**
   * Calculates expected wheel speeds based on current chassis motion. Used for
   * skid detection
   * comparison.
   *
   * @param chassisSpeeds Current chassis speeds
   * @return Array of expected wheel speeds for each module
   */
  private double[] calculateExpectedWheelSpeeds(ChassisSpeeds chassisSpeeds) {
    // Convert chassis speeds to module states
    SwerveModuleState[] expectedStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    double[] expectedSpeeds = new double[4];
    for (int i = 0; i < 4; i++) {
      expectedSpeeds[i] = Math.abs(expectedStates[i].speedMetersPerSecond);
    }

    return expectedSpeeds;
  }

  /**
   * Gets the current skidding ratio calculated by the traction control system.
   * Values > 1.2
   * typically indicate wheel slip.
   *
   * @return Current skidding ratio
   */
  public double getSkiddingRatio() {
    return skiddingRatio;
  }

  /**
   * Checks if any module is currently slipping.
   *
   * @return True if any module is detected as slipping
   */
  public boolean isAnyModuleSlipping() {
    for (boolean slipping : moduleSlipping) {
      if (slipping)
        return true;
    }
    return false;
  }

  /**
   * Gets the current traction control multipliers for all modules.
   *
   * @return Array of power multipliers (1.0 = full power, lower = reduced power)
   */
  public double[] getTractionControlMultipliers() {
    return tractionControlMultipliers.clone();
  }

  /**
   * Enables or disables the traction control system.
   *
   * @param enabled True to enable traction control
   */
  public void setTractionControlEnabled(boolean enabled) {
    TRACTION_CONTROL_ENABLED = enabled;
    if (!enabled) {
      // Reset all multipliers when disabled
      for (int i = 0; i < 4; i++) {
        tractionControlMultipliers[i] = 1.0;
        moduleSlipping[i] = false;
      }
    }
  }

  /**
   * Runs the drive at the desired velocity using Motion Magic motion profiles.
   * This provides
   * smoother acceleration and deceleration compared to direct velocity control.
   *
   * @param speeds Chassis speeds in meters/sec
   */
  public void runVelocityWithMotionMagic(ChassisSpeeds speeds) {
    // Apply drift compensation
    speeds = applyDriftCompensation(speeds);

    // Convert to discrete time for better accuracy
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, DISCRETIZATION_TIME_SECONDS);

    // Calculate module setpoints
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);

    // Enforce velocity limits
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Capture rotation input for drift mode
    double rotationInput = speeds.omegaRadiansPerSecond / getMaxAngularSpeedRadPerSec();

    // Apply drift mode or regular optimization BEFORE sending to modules
    if (RobotContainer.isDriftModeActive) {
      // Apply drift mode adjustments
      applyDriftModeAdjustments(setpointStates, rotationInput);
    } else {
      // Apply regular optimization to prevent module flipping
      for (int i = 0; i < 4; i++) {
        setpointStates[i] = optimizeSwerveModuleState(modules[i].getAngle(), setpointStates[i]);
      }
    }

    // Apply traction control multipliers to setpoints
    if (TRACTION_CONTROL_ENABLED) {
      for (int i = 0; i < 4; i++) {
        setpointStates[i].speedMetersPerSecond *= tractionControlMultipliers[i];
      }
    }

    // Send optimized setpoints to modules using Motion Magic
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpointWithMotionMagic(setpointStates[i]);
    }

    // Log optimized setpoints
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Chassis speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Apply drift compensation
    speeds = applyDriftCompensation(speeds);

    // Convert to discrete time for better accuracy
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, DISCRETIZATION_TIME_SECONDS);

    // Calculate module setpoints
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);

    // Enforce velocity limits
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Capture rotation input for drift mode
    double rotationInput = speeds.omegaRadiansPerSecond / getMaxAngularSpeedRadPerSec();

    // Apply drift mode or regular optimization BEFORE sending to modules
    if (RobotContainer.isDriftModeActive) {
      // Apply drift mode adjustments
      applyDriftModeAdjustments(setpointStates, rotationInput);
    } else {
      // Apply regular optimization to prevent module flipping
      for (int i = 0; i < 4; i++) {
        setpointStates[i] = optimizeSwerveModuleState(modules[i].getAngle(), setpointStates[i]);
      }
    }

    // Apply traction control multipliers to setpoints
    if (TRACTION_CONTROL_ENABLED) {
      for (int i = 0; i < 4; i++) {
        setpointStates[i].speedMetersPerSecond *= tractionControlMultipliers[i];
      }
    }

    // Send optimized setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Custom optimization method to prevent modules from flipping 180 degrees. This
   * ensures smoother
   * direction changes, especially important for drift mode.
   *
   * @param currentAngle The current module angle
   * @param desiredState The desired module state
   * @return An optimized module state that avoids flipping
   */
  private SwerveModuleState optimizeSwerveModuleState(
      Rotation2d currentAngle, SwerveModuleState desiredState) {
    // Handle zero velocity case - maintain current angle to prevent unnecessary
    // movement
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      return new SwerveModuleState(0, currentAngle);
    }

    double targetAngle = desiredState.angle.getDegrees();
    double currentAngleDegrees = currentAngle.getDegrees();

    // Normalize angles to range (-180, 180]
    targetAngle = targetAngle % 360;
    if (targetAngle > 180)
      targetAngle -= 360;
    if (targetAngle <= -180)
      targetAngle += 360;

    currentAngleDegrees = currentAngleDegrees % 360;
    if (currentAngleDegrees > 180)
      currentAngleDegrees -= 360;
    if (currentAngleDegrees <= -180)
      currentAngleDegrees += 360;

    // Calculate angle difference (shortest path)
    double deltaAngle = targetAngle - currentAngleDegrees;
    if (deltaAngle > 180)
      deltaAngle -= 360;
    if (deltaAngle < -180)
      deltaAngle += 360;

    // If change would be greater than 90 degrees, flip direction and angle
    if (Math.abs(deltaAngle) > 90) {
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          Rotation2d.fromDegrees(targetAngle + (deltaAngle > 90 ? -180 : 180)));
    }

    // Otherwise, use the desired state but target the exact angle
    return new SwerveModuleState(
        desiredState.speedMetersPerSecond, Rotation2d.fromDegrees(targetAngle));
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
    resetSimulationPoseCallBack.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
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
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  private double getAngularVelocity() {
    return Units.degreesToRadians(gyroInputs.yawVelocityRadPerSec);
  }

  /**
   * Determines if a vision pose should be rejected based on various criteria.
   * Uses comprehensive
   * filtering to ensure pose estimation quality.
   *
   * @param latestResult The AprilTag detection result
   * @return True if the pose should be rejected, false otherwise
   */
  public boolean shouldRejectPose(AprilTagResult latestResult) {
    // Always accept poses when disabled
    if (DriverStation.isDisabled()) {
      LoggingUtil.logString("Drive/Vision/RejectReason", "ROBOT_DISABLED_ACCEPT_ALL");
      return false;
    }

    // Reject if rotating too quickly (camera blur and pose instability)
    double currentYawRate = Math.abs(Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec));
    if (currentYawRate >= MAX_YAW_RATE_DEGREES_PER_SEC) {
      LoggingUtil.logString("Drive/Vision/RejectReason", "ROTATION_TOO_FAST");
      LoggingUtil.logDouble("Drive/Vision/YawRateAtRejection", currentYawRate);
      return true;
    }

    // Reject if outside field bounds (clearly impossible poses)
    if (isOutsideFieldBounds(latestResult.pose)) {
      LoggingUtil.logString("Drive/Vision/RejectReason", "OUTSIDE_FIELD_BOUNDS");
      LoggingUtil.logDouble("Drive/Vision/RejectedPoseX", latestResult.pose.getX());
      LoggingUtil.logDouble("Drive/Vision/RejectedPoseY", latestResult.pose.getY());
      return true;
    }

    // Reject if ambiguity is too high (unreliable detection)
    if (latestResult.ambiguity > VISION_AMBIGUITY_THRESHOLD) {
      LoggingUtil.logString("Drive/Vision/RejectReason", "HIGH_AMBIGUITY");
      LoggingUtil.logDouble("Drive/Vision/AmbiguityAtRejection", latestResult.ambiguity);
      return true;
    }

    // Reject if distance is too far (measurement uncertainty increases)
    if (latestResult.distToTag > VISION_MAX_DISTANCE_METERS) {
      LoggingUtil.logString("Drive/Vision/RejectReason", "DISTANCE_TOO_FAR");
      LoggingUtil.logDouble("Drive/Vision/DistanceAtRejection", latestResult.distToTag);
      return true;
    }

    // Reject if pose difference is too large (likely false detection)
    Pose2d currentPose = getPose();
    double poseDifference = currentPose.getTranslation().getDistance(latestResult.pose.getTranslation());
    if (poseDifference > VISION_MAX_POSE_DIFFERENCE_METERS) {
      LoggingUtil.logString("Drive/Vision/RejectReason", "POSE_DIFFERENCE_TOO_LARGE");
      LoggingUtil.logDouble("Drive/Vision/PoseDifferenceAtRejection", poseDifference);
      return true;
    }

    // Accept the pose - all checks passed
    LoggingUtil.logString("Drive/Vision/RejectReason", "ACCEPTED");
    return false;
  }

  /** Checks if a pose is outside the field boundaries. */
  private boolean isOutsideFieldBounds(Pose2d pose) {
    return pose.getX() < -FIELD_BORDER_MARGIN_METERS
        || pose.getX() > FIELD_WIDTH_METERS + FIELD_BORDER_MARGIN_METERS
        || pose.getY() < -FIELD_BORDER_MARGIN_METERS
        || pose.getY() > FIELD_HEIGHT_METERS + FIELD_BORDER_MARGIN_METERS;
  }

  /**
   * Converts wheel rotations to meters.
   *
   * @param wheelRotations The number of wheel rotations
   * @return The equivalent distance in meters
   */
  public static double rotationsToMeters(double wheelRotations) {
    return wheelRotations * TunerConstants.FrontLeft.WheelRadius * 2 * Math.PI;
  }

  /**
   * Calculates the distance from the reef edge.
   *
   * @return The distance in meters
   */
  public double distanceFromReefEdge() {
    // Calculate translation with robot offset
    Translation2d robotTranslation = getPose()
        .getTranslation()
        .plus(new Translation2d(ROBOT_REEF_OFFSET_METERS, getPose().getRotation()));

    // Determine reef center based on alliance
    double reefCenterX = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? BLUE_REEF_CENTER_X
        : RED_REEF_CENTER_X;

    // Calculate distance to reef center and subtract reef radius
    double distToReefCenter = robotTranslation.getDistance(new Translation2d(reefCenterX, REEF_CENTER_Y));

    return distToReefCenter - REEF_CENTER_RADIUS;
  }

  /**
   * Calculates the elevator height based on the distance from the reef edge.
   *
   * @return The height in meters
   */
  public double getElevatorHeight() {
    // Use trigonometry to calculate height based on distance
    return distanceFromReefEdge() * (1 / Rotation2d.fromDegrees(ELEVATOR_ANGLE_DEGREES).getTan());
  }

  /**
   * Updates a module's motor configurations when drift mode is toggled.
   *
   * @param moduleIndex The index of the module to update (0-3)
   */
  public void updateModuleConfiguration(int moduleIndex) {
    if (moduleIndex >= 0 && moduleIndex < modules.length) {
      // Access the module's ModuleIO
      if (modules[moduleIndex].getIO() instanceof ModuleIOTalonFX moduleTalon) {
        // Update the motor configurations
        moduleTalon.updateDriftModeConfiguration();
      }
    }
  }

  /**
   * Applies car-like drift mode adjustments to create realistic oversteer
   * behavior. This mimics a
   * rear-wheel drive car with front wheels for steering and rear wheels for
   * power.
   *
   * @param throttleInput The throttle input (0.0 to 1.0)
   * @param steeringInput The steering input (-1.0 to 1.0, left to right)
   */
  public void runCarDriftMode(double throttleInput, double steeringInput) {
    if (!RobotContainer.isDriftModeActive) {
      return;
    }

    // Module indices: FL=0, FR=1, BL=2, BR=3
    int frontLeft = 0;
    int frontRight = 1;
    int rearLeft = 2;
    int rearRight = 3;

    // Calculate front wheel steering angle (limited to ±70 degrees)
    double frontSteeringAngle = MathUtil.clamp(
        steeringInput * DRIFT_FRONT_MAX_STEERING_DEGREES * DRIFT_STEERING_SENSITIVITY,
        -DRIFT_FRONT_MAX_STEERING_DEGREES,
        DRIFT_FRONT_MAX_STEERING_DEGREES);

    // Set front wheels to steering angle with no power (coast)
    modules[frontLeft].runSetpoint(
        new SwerveModuleState(
            DRIFT_FRONT_POWER_MULTIPLIER, Rotation2d.fromDegrees(frontSteeringAngle)));

    modules[frontRight].runSetpoint(
        new SwerveModuleState(
            DRIFT_FRONT_POWER_MULTIPLIER, Rotation2d.fromDegrees(frontSteeringAngle)));

    // Calculate rear wheel speed (high power for oversteer)
    double rearSpeed = throttleInput * getMaxLinearSpeedMetersPerSec() * DRIFT_REAR_POWER_MULTIPLIER;

    // Set rear wheels to forward direction with slight toe-in and high power
    modules[rearLeft].runSetpoint(
        new SwerveModuleState(
            rearSpeed, Rotation2d.fromDegrees(-DRIFT_REAR_TOE_IN_DEGREES))); // Slight toe-in left

    modules[rearRight].runSetpoint(
        new SwerveModuleState(
            rearSpeed, Rotation2d.fromDegrees(DRIFT_REAR_TOE_IN_DEGREES))); // Slight toe-in right

    // Log drift mode telemetry
    Logger.recordOutput("Drive/DriftMode/ThrottleInput", throttleInput);
    Logger.recordOutput("Drive/DriftMode/SteeringInput", steeringInput);
    Logger.recordOutput("Drive/DriftMode/FrontSteeringAngle", frontSteeringAngle);
    Logger.recordOutput("Drive/DriftMode/RearSpeed", rearSpeed);
  }

  /**
   * Legacy drift mode adjustments - kept for compatibility but not used in car
   * mode. Use
   * runCarDriftMode() instead for realistic car-like behavior.
   */
  private void applyDriftModeAdjustments(SwerveModuleState[] setpointStates, double rotationInput) {
    // This method is now deprecated in favor of runCarDriftMode()
    // Left empty to maintain compatibility with existing runVelocity() calls
  }

  /** Logs comprehensive drive system state information */
  private void logDriveSystemState() {
    // Basic drive state
    LoggingUtil.logSubsystemStatus(
        "Drive",
        !DriverStation.isDisabled(),
        RobotContainer.isDriftModeActive ? "DRIFT_MODE" : "NORMAL_MODE");

    // Pose and odometry
    Pose2d currentPose = getPose();
    LoggingUtil.logDouble("Drive/Pose/X", currentPose.getX());
    LoggingUtil.logDouble("Drive/Pose/Y", currentPose.getY());
    LoggingUtil.logDouble("Drive/Pose/Rotation", currentPose.getRotation().getDegrees());

    // Chassis speeds
    ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    LoggingUtil.logDouble("Drive/ChassisSpeed/VX", chassisSpeeds.vxMetersPerSecond);
    LoggingUtil.logDouble("Drive/ChassisSpeed/VY", chassisSpeeds.vyMetersPerSecond);
    LoggingUtil.logDouble(
        "Drive/ChassisSpeed/Omega", Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));
    LoggingUtil.logDouble(
        "Drive/ChassisSpeed/Magnitude",
        Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));

    // Module states
    SwerveModuleState[] states = getModuleStates();
    for (int i = 0; i < states.length; i++) {
      String moduleName = getModuleName(i);
      LoggingUtil.logDouble(
          "Drive/Modules/" + moduleName + "/Speed", states[i].speedMetersPerSecond);
      LoggingUtil.logDouble("Drive/Modules/" + moduleName + "/Angle", states[i].angle.getDegrees());
    }

    // Gyro information
    LoggingUtil.logDouble("Drive/Gyro/Yaw", getRotation().getDegrees());
    // Note: Pitch and Roll may not be available on all gyros
    // LoggingUtil.logDouble("Drive/Gyro/Pitch",
    // gyroInputs.pitchPosition.getDegrees());
    // LoggingUtil.logDouble("Drive/Gyro/Roll",
    // gyroInputs.rollPosition.getDegrees());
    LoggingUtil.logDouble("Drive/Gyro/YawRate", Math.toDegrees(gyroInputs.yawVelocityRadPerSec));
    LoggingUtil.logBoolean("Drive/Gyro/Connected", gyroInputs.connected);

    // Vision system state
    for (int i = 0; i < limelights.length; i++) {
      LoggingUtil.logBoolean("Drive/Vision/Limelight" + i + "/Active", limeLightsActive);
    }

    // Drift compensation
    LoggingUtil.logBoolean("Drive/DriftCompensation/Enabled", isDriftCompensationEnabled);
    LoggingUtil.logDouble("Drive/DriftCompensation/X", driftCompensationX);
    LoggingUtil.logDouble("Drive/DriftCompensation/Y", driftCompensationY);
    LoggingUtil.logBoolean("Drive/DriftCompensation/Calibrating", isDriftCalibrationActive);

    // Traction control detailed logging
    LoggingUtil.logBoolean("Drive/TractionControl/Enabled", TRACTION_CONTROL_ENABLED);
    LoggingUtil.logDouble("Drive/TractionControl/SkiddingRatio", skiddingRatio);
    LoggingUtil.logBoolean("Drive/TractionControl/AnySlipping", isAnyModuleSlipping());
    LoggingUtil.logDoubleArray("Drive/TractionControl/Multipliers", tractionControlMultipliers);
    LoggingUtil.logDoubleArray("Drive/TractionControl/FilteredSpeeds", filteredWheelSpeeds);

    // Individual module slip status
    for (int i = 0; i < moduleSlipping.length; i++) {
      LoggingUtil.logBoolean("Drive/TractionControl/Module" + i + "Slipping", moduleSlipping[i]);
    }

    // PathPlanner state
    LoggingUtil.logDouble("Drive/PathPlanner/TranslationP", PP_TRANSLATION_P);
    LoggingUtil.logDouble("Drive/PathPlanner/RotationP", PP_ROTATION_P);
  }

  /** Updates all tunable parameters from SmartDashboard */
  private void updateTunableParameters() {
    // PathPlanner PID tuning
    PP_TRANSLATION_P = LoggingUtil.getTunableDouble("Drive/PathPlanner/TranslationP", PP_TRANSLATION_P);
    PP_TRANSLATION_I = LoggingUtil.getTunableDouble("Drive/PathPlanner/TranslationI", PP_TRANSLATION_I);
    PP_TRANSLATION_D = LoggingUtil.getTunableDouble("Drive/PathPlanner/TranslationD", PP_TRANSLATION_D);
    PP_ROTATION_P = LoggingUtil.getTunableDouble("Drive/PathPlanner/RotationP", PP_ROTATION_P);
    PP_ROTATION_I = LoggingUtil.getTunableDouble("Drive/PathPlanner/RotationI", PP_ROTATION_I);
    PP_ROTATION_D = LoggingUtil.getTunableDouble("Drive/PathPlanner/RotationD", PP_ROTATION_D);

    // Vision tuning parameters
    xyStdDevCoeff = LoggingUtil.getTunableDouble("Drive/Vision/XYStdDevCoeff", xyStdDevCoeff);
    rStdDevCoeff = LoggingUtil.getTunableDouble("Drive/Vision/RStdDevCoeff", rStdDevCoeff);

    // Enhanced vision filtering parameters
    VISION_AMBIGUITY_THRESHOLD = LoggingUtil.getTunableDouble("Drive/Vision/AmbiguityThreshold",
        VISION_AMBIGUITY_THRESHOLD);
    VISION_MAX_DISTANCE_METERS = LoggingUtil.getTunableDouble("Drive/Vision/MaxDistanceMeters",
        VISION_MAX_DISTANCE_METERS);
    VISION_MAX_POSE_DIFFERENCE_METERS = LoggingUtil.getTunableDouble(
        "Drive/Vision/MaxPoseDifferenceMeters", VISION_MAX_POSE_DIFFERENCE_METERS);

    // Drive base radius for characterization
    LoggingUtil.logDouble("Drive/Characterization/DriveBaseRadius", DRIVE_BASE_RADIUS);

    // Field position limits
    LoggingUtil.logDouble("Drive/Field/Width", FIELD_WIDTH_METERS);
    LoggingUtil.logDouble("Drive/Field/Height", FIELD_HEIGHT_METERS);

    // Drift mode parameters
    LoggingUtil.logBoolean("Drive/DriftMode/Active", RobotContainer.isDriftModeActive);
    LoggingUtil.logDouble("Drive/DriftMode/FrontMaxSteering", DRIFT_FRONT_MAX_STEERING_DEGREES);
    LoggingUtil.logDouble("Drive/DriftMode/RearPowerMultiplier", DRIFT_REAR_POWER_MULTIPLIER);
  }

  /**
   * Gets a human-readable name for a swerve module
   *
   * @param index Module index (0-3)
   * @return Module name (FL, FR, BL, BR)
   */
  private String getModuleName(int index) {
    switch (index) {
      case 0:
        return "FL";
      case 1:
        return "FR";
      case 2:
        return "BL";
      case 3:
        return "BR";
      default:
        return "Unknown";
    }
  }

  /**
   * Updates odometry using wheel positions and gyro data. Handles high-frequency
   * sampling and
   * provides comprehensive logging with performance optimization.
   */
  private void updateOdometry() {
    PerformanceMonitor monitor = PerformanceMonitor.getInstance();
    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;

    // Log sample count only when performance allows
    if (!monitor.shouldSkipNonCriticalOperations("Drive")) {
      LoggingUtil.logDouble("Drive/Odometry/SampleCount", sampleCount);
      LoggingUtil.logDouble("Drive/Odometry/Frequency", sampleCount / 0.02); // Assuming 20ms loop
    }

    for (int i = 0; i < sampleCount; i++) {
      // Read current module positions
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

      // Update gyro rotation with fallback to kinematics
      updateGyroRotation(moduleDeltas, i);

      // Apply odometry update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

      // Log odometry data with adaptive frequency
      if (!monitor.shouldSkipNonCriticalOperations("Drive") && shouldLogOdometryDetails()) {
        logOdometryData(sampleTimestamps[i], modulePositions, moduleDeltas);
      }
    }
  }

  /**
   * Updates gyro rotation with fallback to kinematics-based estimation.
   *
   * @param moduleDeltas The change in module positions since last update
   * @param sampleIndex  The current sample index
   */
  private void updateGyroRotation(SwerveModulePosition[] moduleDeltas, int sampleIndex) {
    if (gyroInputs.connected) {
      // Use real gyro data when available
      rawGyroRotation = gyroInputs.odometryYawPositions[sampleIndex];
      LoggingUtil.logString("Drive/Odometry/RotationSource", "GYRO");
    } else {
      // Fall back to kinematics-based rotation estimation
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      LoggingUtil.logString("Drive/Odometry/RotationSource", "KINEMATICS");
      LoggingUtil.logDouble("Drive/Odometry/KinematicsDelta", Math.toDegrees(twist.dtheta));
    }
  }

  /**
   * Logs comprehensive odometry data for debugging and analysis.
   *
   * @param timestamp       The timestamp of this odometry sample
   * @param modulePositions Current module positions
   * @param moduleDeltas    Changes in module positions
   */
  private void logOdometryData(
      double timestamp,
      SwerveModulePosition[] modulePositions,
      SwerveModulePosition[] moduleDeltas) {
    // Log current pose
    Pose2d currentPose = getPose();
    LoggingUtil.logDouble("Drive/Odometry/PoseX", currentPose.getX());
    LoggingUtil.logDouble("Drive/Odometry/PoseY", currentPose.getY());
    LoggingUtil.logDouble("Drive/Odometry/PoseRotation", currentPose.getRotation().getDegrees());

    // Log velocities
    LoggingUtil.logDouble("Drive/Odometry/LinearVelocity", getLinearVelocity());
    LoggingUtil.logDouble(
        "Drive/Odometry/AngularVelocity", Math.toDegrees(getAngularVelocityRadPerSec()));

    // Log module deltas for debugging wheel slip
    double[] wheelDeltas = new double[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] = moduleDeltas[i].distanceMeters;
    }
    LoggingUtil.logDoubleArray("Drive/Odometry/WheelDeltas", wheelDeltas);

    // Log timestamp information
    LoggingUtil.logDouble("Drive/Odometry/Timestamp", timestamp);
  }

  /**
   * Updates vision measurements from all limelights with comprehensive filtering
   * and logging.
   * Optimized for performance with adaptive frequency. Processes AprilTag
   * detections and applies
   * them to pose estimation with calculated standard deviations.
   */
  private void updateVisionMeasurements() {
    PerformanceMonitor monitor = PerformanceMonitor.getInstance();

    if (!limeLightsActive) {
      LoggingUtil.logString("Drive/Vision/Status", "DISABLED");
      return;
    }

    // Get results from all limelights
    AprilTagResult[] visionResults = getVisionResults();

    // Skip detailed logging if performance is critical
    if (!monitor.shouldSkipNonCriticalOperations("Drive")) {
      logRawVisionData(visionResults);
    }

    // Process each valid result
    int acceptedMeasurements = 0;
    int rejectedMeasurements = 0;

    for (int i = 0; i < visionResults.length; i++) {
      AprilTagResult result = visionResults[i];
      if (result != null) {
        String limelightName = getLimelightName(i);

        if (shouldRejectPose(result)) {
          rejectedMeasurements++;
          // Skip detailed rejection logging in degraded mode
          if (!monitor.shouldSkipNonCriticalOperations("Drive")) {
            logRejectedVisionMeasurement(limelightName, result);
          }
        } else {
          acceptedMeasurements++;
          processAcceptedVisionMeasurement(limelightName, result);
        }
      }
    }

    // Always log summary statistics (critical for debugging)
    LoggingUtil.logDouble("Drive/Vision/AcceptedMeasurements", acceptedMeasurements);
    LoggingUtil.logDouble("Drive/Vision/RejectedMeasurements", rejectedMeasurements);
    LoggingUtil.logString(
        "Drive/Vision/Status", acceptedMeasurements > 0 ? "ACTIVE" : "NO_VALID_TARGETS");
  }

  /**
   * Gets vision results from all limelights.
   *
   * @return Array of AprilTag results, null entries indicate no detection
   */
  private AprilTagResult[] getVisionResults() {
    return new AprilTagResult[] {
        limelight_a.getEstimate().orElse(null),
        limelight_b.getEstimate().orElse(null),
        limelight_c.getEstimate().orElse(null)
    };
  }

  /**
   * Logs raw vision data from all limelights for debugging.
   *
   * @param results Array of vision results from limelights
   */
  private void logRawVisionData(AprilTagResult[] results) {
    for (int i = 0; i < results.length; i++) {
      String limelightName = getLimelightName(i);
      AprilTagResult result = results[i];

      if (result != null) {
        LoggingUtil.logDouble("Drive/Vision/" + limelightName + "/PoseX", result.pose.getX());
        LoggingUtil.logDouble("Drive/Vision/" + limelightName + "/PoseY", result.pose.getY());
        LoggingUtil.logDouble(
            "Drive/Vision/" + limelightName + "/PoseRotation",
            result.pose.getRotation().getDegrees());
        LoggingUtil.logDouble("Drive/Vision/" + limelightName + "/Distance", result.distToTag);
        LoggingUtil.logDouble("Drive/Vision/" + limelightName + "/Ambiguity", result.ambiguity);
        LoggingUtil.logDouble("Drive/Vision/" + limelightName + "/TagCount", result.tagCount);
        LoggingUtil.logDouble("Drive/Vision/" + limelightName + "/Timestamp", result.time);
        LoggingUtil.logBoolean("Drive/Vision/" + limelightName + "/HasTarget", true);
      } else {
        LoggingUtil.logBoolean("Drive/Vision/" + limelightName + "/HasTarget", false);
      }
    }
  }

  /**
   * Processes an accepted vision measurement and adds it to pose estimation.
   *
   * @param limelightName The name of the limelight providing the measurement
   * @param result        The AprilTag detection result
   */
  private void processAcceptedVisionMeasurement(String limelightName, AprilTagResult result) {
    // Calculate standard deviations based on distance, tag count, and ambiguity
    VisionStandardDeviations stdDevs = calculateVisionStandardDeviations(result);

    // Add measurement to pose estimator
    addVisionMeasurement(
        result.pose,
        result.time,
        VecBuilder.fill(stdDevs.xyStdDev, stdDevs.xyStdDev, stdDevs.rStdDev));

    // Log acceptance and calculated values
    LoggingUtil.logString("Drive/Vision/" + limelightName + "/Status", "ACCEPTED");
    LoggingUtil.logDouble("Drive/Vision/" + limelightName + "/XYStdDev", stdDevs.xyStdDev);
    LoggingUtil.logDouble("Drive/Vision/" + limelightName + "/RStdDev", stdDevs.rStdDev);
    LoggingUtil.logDouble("Drive/Vision/" + limelightName + "/EffectiveDistance", result.distToTag);

    // Calculate and log pose difference for analysis
    Pose2d currentPose = getPose();
    double poseDifference = currentPose.getTranslation().getDistance(result.pose.getTranslation());
    double rotationDifference = Math.abs(currentPose.getRotation().minus(result.pose.getRotation()).getDegrees());

    LoggingUtil.logDouble("Drive/Vision/" + limelightName + "/PoseDifference", poseDifference);
    LoggingUtil.logDouble(
        "Drive/Vision/" + limelightName + "/RotationDifference", rotationDifference);
  }

  /**
   * Logs information about rejected vision measurements for debugging.
   *
   * @param limelightName The name of the limelight providing the measurement
   * @param result        The rejected AprilTag detection result
   */
  private void logRejectedVisionMeasurement(String limelightName, AprilTagResult result) {
    LoggingUtil.logString("Drive/Vision/" + limelightName + "/Status", "REJECTED");

    // Log rejection reasons
    boolean tooFastRotation = Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec) >= MAX_YAW_RATE_DEGREES_PER_SEC;
    boolean outsideBounds = isOutsideFieldBounds(result.pose);

    LoggingUtil.logBoolean(
        "Drive/Vision/" + limelightName + "/RejectedTooFastRotation", tooFastRotation);
    LoggingUtil.logBoolean(
        "Drive/Vision/" + limelightName + "/RejectedOutsideBounds", outsideBounds);

    if (tooFastRotation) {
      LoggingUtil.logDouble(
          "Drive/Vision/" + limelightName + "/CurrentYawRate",
          Math.abs(Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec)));
      LoggingUtil.logDouble(
          "Drive/Vision/" + limelightName + "/MaxYawRate", MAX_YAW_RATE_DEGREES_PER_SEC);
    }
  }

  /**
   * Calculates vision measurement standard deviations based on detection quality.
   * Uses the same
   * proven formula as before, but encapsulated for clarity.
   *
   * @param result The AprilTag detection result
   * @return Standard deviations for X/Y and rotation measurements
   */
  private VisionStandardDeviations calculateVisionStandardDeviations(AprilTagResult result) {
    // Use the existing tuned formula - preserving all current constants
    double xyStdDev = xyStdDevCoeff
        * Math.max(Math.pow(result.distToTag, 2.0), 0.5)
        / result.tagCount
        * Math.sqrt(result.ambiguity)
        * sdMultiplier;

    double rStdDev = rStdDevCoeff
        * Math.max(Math.pow(result.distToTag, 2.0), 0.5)
        / result.tagCount
        * Math.sqrt(result.ambiguity)
        * sdMultiplier;

    return new VisionStandardDeviations(xyStdDev, rStdDev);
  }

  /**
   * Gets the name of a limelight by index.
   *
   * @param index Limelight index (0-2)
   * @return Limelight name
   */
  private String getLimelightName(int index) {
    switch (index) {
      case 0:
        return "LimelightA";
      case 1:
        return "LimelightB";
      case 2:
        return "LimelightC";
      default:
        return "Unknown";
    }
  }

  /**
   * Gets the angular velocity in radians per second.
   *
   * @return Angular velocity in rad/s
   */
  private double getAngularVelocityRadPerSec() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /**
   * Determines if vision processing should be updated this cycle based on
   * performance. Implements
   * adaptive frequency to maintain target loop times.
   *
   * @return True if vision should be updated, false to skip
   */
  private boolean shouldUpdateVision() {
    PerformanceMonitor monitor = PerformanceMonitor.getInstance();

    // Always update vision when disabled (for alignment)
    if (DriverStation.isDisabled()) {
      return true;
    }

    // Skip vision updates if in degraded mode and not critical
    if (monitor.isDegradedModeActive()) {
      // Update vision every 4th cycle in degraded mode (12.5Hz instead of 50Hz)
      return Timer.getFPGATimestamp() % 0.08 < 0.02;
    }

    // Normal operation - update vision every cycle
    return true;
  }

  /**
   * Updates vision systems (limelights) with robot orientation. Separated from
   * vision measurements
   * for performance optimization.
   */
  private void updateVisionSystems() {
    for (LimeLightCam limelight : limelights) {
      limelight.SetRobotOrientation(getPose().getRotation());
    }
  }

  /**
   * Updates only critical dashboard values when in performance-critical mode.
   * Skips non-essential
   * tuning parameters to save processing time.
   */
  private void updateCriticalDashboardValues() {
    // Only update critical safety and control parameters
    PerformanceMonitor monitor = PerformanceMonitor.getInstance();

    // Update essential PID values for auto scoring (critical for competition)
    AutoScoreCommand.X_PID_P = SmartDashboard.getNumber("X_PID_P", AutoScoreCommand.X_PID_P);
    AutoScoreCommand.Y_PID_P = SmartDashboard.getNumber("Y_PID_P", AutoScoreCommand.Y_PID_P);
    AutoScoreCommand.THETA_PID_P = SmartDashboard.getNumber("THETA_PID_P", AutoScoreCommand.THETA_PID_P);

    // Update traction control enable/disable (safety critical)
    TRACTION_CONTROL_ENABLED = SmartDashboard.getBoolean("TractionControl/Enabled", TRACTION_CONTROL_ENABLED);
    SmartDashboard.putBoolean("TractionControl/Enabled", TRACTION_CONTROL_ENABLED);

    // Update performance monitoring status
    SmartDashboard.putBoolean("Performance/DegradedMode", monitor.isDegradedModeActive());
    SmartDashboard.putNumber("Performance/AverageLoopTime", monitor.getAverageLoopTime());
  }

  /**
   * Determines if detailed odometry logging should occur based on performance.
   *
   * @return True if detailed logging should occur
   */
  private boolean shouldLogOdometryDetails() {
    PerformanceMonitor monitor = PerformanceMonitor.getInstance();

    if (monitor.isDegradedModeActive()) {
      // Log only every 10th update in degraded mode
      return Timer.getFPGATimestamp() % 0.2 < 0.02;
    }

    return true;
  }

  /** Simple data class for holding vision standard deviations. */
  private static class VisionStandardDeviations {
    public final double xyStdDev;
    public final double rStdDev;

    public VisionStandardDeviations(double xyStdDev, double rStdDev) {
      this.xyStdDev = xyStdDev;
      this.rStdDev = rStdDev;
    }
  }
}

/** Custom exception class for specific error handling. */
class InvalidRobotNameException extends RuntimeException {
  String[] invalidStrings = {
      "Elevator", "Drive", "ControllerLogging", "LocalADStarAK", "PLog", "IntakeIOTalonFX",
  };

  /** Constructs a new exception with custom stack trace. */
  public InvalidRobotNameException(String message) {
    super(message);
    // Select a random string from the array
    String invalidString = invalidStrings[(int) (Math.random() * invalidStrings.length)];

    // Create a fake stack trace with the random string
    StackTraceElement[] stack = new StackTraceElement[] {
        new StackTraceElement(
            invalidString, "[null]", invalidString + ".java", (int) (Math.random() * 500))
    };
    this.setStackTrace(stack);
  }
}
