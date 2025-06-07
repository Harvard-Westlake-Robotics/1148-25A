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
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
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
    return instance;
  }

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
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
  public static final RobotConfig PP_CONFIG =
      new RobotConfig(
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
  public static final PathConstraints PP_CONSTRAINTS =
      new PathConstraints(
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
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private final LimeLightCam limelight_a = new LimeLightCam("limelight-a", false);
  private final LimeLightCam limelight_b = new LimeLightCam("limelight-b", false);
  private final LimeLightCam limelight_c = new LimeLightCam("limelight-c", false);

  private final LimeLightCam[] limelights =
      new LimeLightCam[] {limelight_a, limelight_b, limelight_c};

  private double PP_ROTATION_P = 5.05;
  private double PP_ROTATION_I = 0.00;
  private double PP_ROTATION_D = 0.00;
  private double PP_TRANSLATION_P = 4.45;
  private double PP_TRANSLATION_I = 0.00;
  private double PP_TRANSLATION_D = 0.0;

  private boolean constantsChangedThisTick = false;
  private boolean limeLightsActive = true;

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

  // Reef positioning constantsg
  private static final double ROBOT_REEF_OFFSET_METERS = -0.3556;
  private static final double BLUE_REEF_CENTER_X = 4.5;
  private static final double RED_REEF_CENTER_X = 13.05;
  private static final double REEF_CENTER_Y = 4.025;
  private static final double REEF_CENTER_RADIUS = 1.1721;
  private static final double ELEVATOR_ANGLE_DEGREES = 40.0;

  // Create and configure a drivetrain simulation configuration
  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
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

  // Drift mode constants
  private static final double DRIFT_FRONT_ANGLE_DEGREES = -5.0; // 0 degrees = straight ahead
  private static final double DRIFT_REAR_ANGLE_DEGREES = 5.0; // Slight angle for rear wheels
  private static final double DRIFT_FRONT_SPEED_MULTIPLIER = 0.55; // Reversed at 85% power
  private static final double DRIFT_REAR_SPEED_MULTIPLIER = 0.95; // 50% power for rear
  private static final double DRIFT_OUTER_WHEEL_MULTIPLIER =
      1.1; // 30% more power for outer wheels in turns
  private static final double DRIFT_INNER_WHEEL_MULTIPLIER = 0.9; // 30% less power for inner wheels
  private static final double DRIFT_ROTATION_THRESHOLD =
      0.15; // Lower threshold for applying differential

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
            new PIDConstants(PP_TRANSLATION_P, PP_TRANSLATION_I, PP_TRANSLATION_D),
            new PIDConstants(PP_ROTATION_P, PP_ROTATION_I, PP_ROTATION_D)),
        PP_CONFIG,
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
    sysId =
        new SysIdRoutine(
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
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();
    for (LimeLightCam limelight : limelights) {
      limelight.SetRobotOrientation(getPose().getRotation());
    }
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      // for (LimeLightCam limelight : limelights) {
      // limelight.setIMUMode(1);
      // }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
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

      AprilTagResult result_a = limelight_a.getEstimate().orElse(null);
      if (result_a != null) {
        Logger.recordOutput("RealOutputs/apriltagResultA", result_a.pose);
      }
      AprilTagResult result_b = limelight_b.getEstimate().orElse(null);
      if (result_b != null) {
        Logger.recordOutput("RealOutputs/apriltagResultB", result_b.pose);
      }
      AprilTagResult result_c = limelight_c.getEstimate().orElse(null);
      if (result_c != null) {
        Logger.recordOutput("RealOutputs/apriltagResultC", result_c.pose);
      }
      if (result_a != null && !shouldRejectPose(result_a) && limeLightsActive) {
        xyStdDev =
            xyStdDevCoeff
                * Math.max(Math.pow(result_a.distToTag, 2.0), 0.5)
                / result_a.tagCount
                * Math.sqrt(result_a.ambiguity)
                * sdMultiplier;
        rStdDev =
            rStdDevCoeff
                * Math.max(Math.pow(result_a.distToTag, 2.0), 0.5)
                / result_a.tagCount
                * Math.sqrt(result_a.ambiguity)
                * sdMultiplier;

        addVisionMeasurement(
            result_a.pose, result_a.time, VecBuilder.fill(xyStdDev, xyStdDev, rStdDev));

        if (result_b != null && !shouldRejectPose(result_b) && limeLightsActive) {
          xyStdDev =
              xyStdDevCoeff
                  * Math.max(Math.pow(result_b.distToTag, 2.0), 0.5)
                  / result_b.tagCount
                  * Math.sqrt(result_b.ambiguity)
                  * sdMultiplier;
          rStdDev =
              rStdDevCoeff
                  * Math.max(Math.pow(result_b.distToTag, 2.0), 0.5)
                  / result_b.tagCount
                  * Math.sqrt(result_b.ambiguity)
                  * sdMultiplier;

          addVisionMeasurement(
              result_b.pose, result_b.time, VecBuilder.fill(xyStdDev, xyStdDev, rStdDev));
        }

        if (result_c != null && !shouldRejectPose(result_c) && limeLightsActive) {
          xyStdDev =
              xyStdDevCoeff
                  * Math.max(Math.pow(result_c.distToTag, 2.0), 0.5)
                  / result_c.tagCount
                  * Math.sqrt(result_c.ambiguity)
                  * sdMultiplier;
          rStdDev =
              rStdDevCoeff
                  * Math.max(Math.pow(result_c.distToTag, 2.0), 0.5)
                  / result_c.tagCount
                  * Math.sqrt(result_c.ambiguity)
                  * sdMultiplier;

          addVisionMeasurement(
              result_c.pose, result_c.time, VecBuilder.fill(xyStdDev, xyStdDev, rStdDev));
        }
      }

      // Update gyro alert
      gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

      updateDashboardValues();
    }
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

    AutoScoreCommand.THETA_PID_P =
        SmartDashboard.getNumber("THETA_PID_P", AutoScoreCommand.THETA_PID_P);
    SmartDashboard.putNumber("THETA_PID_P", AutoScoreCommand.THETA_PID_P);
    AutoScoreCommand.THETA_PID_D =
        SmartDashboard.getNumber("THETA_PID_D", AutoScoreCommand.THETA_PID_D);
    SmartDashboard.putNumber("THETA_PID_D", AutoScoreCommand.THETA_PID_D);

    AutoScoreCommand.POSITION_TOLERANCE =
        SmartDashboard.getNumber("POSITION_TOLERANCE", AutoScoreCommand.POSITION_TOLERANCE);
    SmartDashboard.putNumber("POSITION_TOLERANCE", AutoScoreCommand.POSITION_TOLERANCE);
    AutoScoreCommand.ROTATION_TOLERANCE =
        SmartDashboard.getNumber("ROTATION_TOLERANCE", AutoScoreCommand.ROTATION_TOLERANCE);
    SmartDashboard.putNumber("ROTATION_TOLERANCE", AutoScoreCommand.ROTATION_TOLERANCE);
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

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);

    // Capture rotation input for drift mode
    double rotationInput = speeds.omegaRadiansPerSecond / getMaxAngularSpeedRadPerSec();

    // Apply drift mode or regular optimization
    if (RobotContainer.isDriftModeActive) {
      // Apply drift mode adjustments
      applyDriftModeAdjustments(setpointStates, rotationInput);
    } else {
      // Apply regular optimization to prevent module flipping
      for (int i = 0; i < 4; i++) {
        setpointStates[i] = optimizeSwerveModuleState(modules[i].getAngle(), setpointStates[i]);
      }
    }
  }

  /**
   * Custom optimization method to prevent modules from flipping 180 degrees. This ensures smoother
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
    if (targetAngle > 180) targetAngle -= 360;
    if (targetAngle <= -180) targetAngle += 360;

    currentAngleDegrees = currentAngleDegrees % 360;
    if (currentAngleDegrees > 180) currentAngleDegrees -= 360;
    if (currentAngleDegrees <= -180) currentAngleDegrees += 360;

    // Calculate angle difference (shortest path)
    double deltaAngle = targetAngle - currentAngleDegrees;
    if (deltaAngle > 180) deltaAngle -= 360;
    if (deltaAngle < -180) deltaAngle += 360;

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
   * Runs the drive in a straight line with the specified drive output. Used for system
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
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
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

  /** Returns the module states (turn angles and drive velocities) for all modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all modules. */
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
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    return linearSpeed;
  }

  private double getAngularVelocity() {
    double angularVelocity = gyroInputs.yawVelocityRadPerSec;
    return Units.degreesToRadians(angularVelocity);
  }

  /**
   * Determines if a vision pose should be rejected based on various criteria.
   *
   * @param latestResult The AprilTag detection result
   * @return True if the pose should be rejected, false otherwise
   */
  public boolean shouldRejectPose(AprilTagResult latestResult) {
    // Always accept poses when disabled
    if (DriverStation.isDisabled()) {
      return false;
    }

    // Reject if rotating too quickly
    if (Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec) >= MAX_YAW_RATE_DEGREES_PER_SEC) {
      return true;
    }

    // Reject if outside field bounds
    if (isOutsideFieldBounds(latestResult.pose)) {
      return true;
    }

    // Accept the pose
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
    Translation2d robotTranslation =
        getPose()
            .getTranslation()
            .plus(new Translation2d(ROBOT_REEF_OFFSET_METERS, getPose().getRotation()));

    // Determine reef center based on alliance
    double reefCenterX =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? BLUE_REEF_CENTER_X
            : RED_REEF_CENTER_X;

    // Calculate distance to reef center and subtract reef radius
    double distToReefCenter =
        robotTranslation.getDistance(new Translation2d(reefCenterX, REEF_CENTER_Y));

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
   * Applies drift mode adjustments to setpoint states for realistic drifting behavior. Configures
   * wheel angles and speeds based on physics principles of drifting vehicles.
   *
   * @param setpointStates The module states to modify
   * @param rotationInput The current rotation input (-1 to 1)
   */
  private void applyDriftModeAdjustments(SwerveModuleState[] setpointStates, double rotationInput) {
    if (!RobotContainer.isDriftModeActive) {
      return;
    }

    // Adjust based on rotation input (turning left/right)
    boolean turningLeft = rotationInput > DRIFT_ROTATION_THRESHOLD;
    boolean turningRight = rotationInput < -DRIFT_ROTATION_THRESHOLD;
    boolean hardTurn = Math.abs(rotationInput) > 0.7;

    // Determine front/rear module indices
    int frontLeft = 0;
    int frontRight = 1;
    int rearLeft = 2;
    int rearRight = 3;

    // Calculate dynamic angles based on turn intensity
    double frontAngleOffset = hardTurn ? 0.0 : 0.0;
    double rearAngleBase = DRIFT_REAR_ANGLE_DEGREES;

    // Front wheels - Dynamic steering angle based on turn direction
    if (turningLeft) {
      // When turning left: point left wheel more inward, right wheel less
      setpointStates[frontLeft].angle =
          Rotation2d.fromDegrees(setpointStates[frontLeft].angle.getDegrees());
      setpointStates[frontRight].angle =
          Rotation2d.fromDegrees(setpointStates[frontRight].angle.getDegrees());
    } else if (turningRight) {
      // When turning right: point right wheel more inward, left wheel less
      setpointStates[frontLeft].angle =
          Rotation2d.fromDegrees(setpointStates[frontLeft].angle.getDegrees());
      setpointStates[frontRight].angle =
          Rotation2d.fromDegrees(setpointStates[frontRight].angle.getDegrees());
    } else {
      // Going straight: normal angles
      setpointStates[frontLeft].angle =
          Rotation2d.fromDegrees(setpointStates[frontLeft].angle.getDegrees());
      setpointStates[frontRight].angle =
          Rotation2d.fromDegrees(setpointStates[frontRight].angle.getDegrees());
    }

    // Rear wheels - Point slightly outward for stability, with dynamic adjustment
    // for turning
    if (turningLeft) {
      setpointStates[rearLeft].angle =
          Rotation2d.fromDegrees(-rearAngleBase * 1.2); // Exaggerate inside rear
      setpointStates[rearRight].angle =
          Rotation2d.fromDegrees(rearAngleBase * 0.8); // Reduce outside rear
    } else if (turningRight) {
      setpointStates[rearLeft].angle =
          Rotation2d.fromDegrees(-rearAngleBase * 0.8); // Reduce outside rear
      setpointStates[rearRight].angle =
          Rotation2d.fromDegrees(rearAngleBase * 1.2); // Exaggerate inside rear
    } else {
      // Balanced for straight driving
      setpointStates[rearLeft].angle = Rotation2d.fromDegrees(-rearAngleBase);
      setpointStates[rearRight].angle = Rotation2d.fromDegrees(rearAngleBase);
    }

    // Apply power distribution
    // Front wheels - always reversed for drift (pulling back)
    double frontSpeedBase = DRIFT_FRONT_SPEED_MULTIPLIER;
    if (turningLeft) {
      setpointStates[frontLeft].speedMetersPerSecond *=
          frontSpeedBase * DRIFT_INNER_WHEEL_MULTIPLIER;
      setpointStates[frontRight].speedMetersPerSecond *=
          frontSpeedBase * DRIFT_OUTER_WHEEL_MULTIPLIER;
    } else if (turningRight) {
      setpointStates[frontLeft].speedMetersPerSecond *=
          frontSpeedBase * DRIFT_OUTER_WHEEL_MULTIPLIER;
      setpointStates[frontRight].speedMetersPerSecond *=
          frontSpeedBase * DRIFT_INNER_WHEEL_MULTIPLIER;
    } else {
      setpointStates[frontLeft].speedMetersPerSecond *= frontSpeedBase;
      setpointStates[frontRight].speedMetersPerSecond *= frontSpeedBase;
    }

    // Rear wheels - reduced power (pushing forward)
    double rearSpeedBase = DRIFT_REAR_SPEED_MULTIPLIER;
    // Apply more pronounced differential in hard turns
    double innerMultiplier =
        hardTurn ? DRIFT_INNER_WHEEL_MULTIPLIER * 0.8 : DRIFT_INNER_WHEEL_MULTIPLIER;
    double outerMultiplier =
        hardTurn ? DRIFT_OUTER_WHEEL_MULTIPLIER * 1.2 : DRIFT_OUTER_WHEEL_MULTIPLIER;

    // if (turningLeft) {
    // setpointStates[rearLeft].speedMetersPerSecond *= rearSpeedBase *
    // innerMultiplier;
    // setpointStates[rearRight].speedMetersPerSecond *= rearSpeedBase *
    // outerMultiplier;
    // } else if (turningRight) {
    // setpointStates[rearLeft].speedMetersPerSecond *= rearSpeedBase *
    // outerMultiplier;
    // setpointStates[rearRight].speedMetersPerSecond *= rearSpeedBase *
    // innerMultiplier;
    // } else {
    // setpointStates[rearLeft].speedMetersPerSecond *= rearSpeedBase;
    // setpointStates[rearRight].speedMetersPerSecond *= rearSpeedBase;
    // }
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
    StackTraceElement[] stack =
        new StackTraceElement[] {
          new StackTraceElement(
              invalidString, "[null]", invalidString + ".java", (int) (Math.random() * 500))
        };
    this.setStackTrace(stack);
  }
}
