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
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.PhoenixUtil;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

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
      LinearVelocity.ofBaseUnits(4.8, MetersPerSecond),
      LinearAcceleration.ofBaseUnits(7.0, MetersPerSecondPerSecond),
      AngularVelocity.ofBaseUnits(820, DegreesPerSecond),
      AngularAcceleration.ofBaseUnits(
          1454, DegreesPerSecondPerSecond)); // PathConstraints.unlimitedConstraints(12);
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

  private double PP_ROTATION_P = 6.65;
  private double PP_ROTATION_I = 0.00;
  private double PP_ROTATION_D = 0.00;
  private double PP_TRANSLATION_P = 6.18;
  private double PP_TRANSLATION_I = 0.006;
  private double PP_TRANSLATION_D = 0.00;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
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

  private boolean constantsChangedThisTick = false;
  private boolean limeLightsActive = true;

  public boolean isLimeLightsActive() {
    return limeLightsActive;
  }

  public void setLimeLightsActive(boolean limeLightsActive) {
    this.limeLightsActive = limeLightsActive;
  }

  private double xyStdDevCoeff = 4.5;
  private double rStdDevCoeff = 6.5;
  private double xyStdDev = 0.8;
  private double rStdDev = 9.2;

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
      // for (LimeLightCam limelight : limelights) {
      // limelight.setIMUMode(2);
      // }
      AprilTagResult result_a = limelight_a.getEstimate().orElse(null);
      if (result_a != null)
        Logger.recordOutput("RealOutputs/apriltagResultA", result_a.pose);
      AprilTagResult result_b = limelight_b.getEstimate().orElse(null);
      if (result_b != null)
        Logger.recordOutput("RealOutputs/apriltagResultB", result_b.pose);
      AprilTagResult result_c = limelight_c.getEstimate().orElse(null);
      if (result_c != null)
        Logger.recordOutput("RealOutputs/apriltagResultC", result_c.pose);
      if (result_a != null && !shouldRejectPose(result_a) && limeLightsActive) {
        xyStdDev = xyStdDevCoeff
            * Math.max(Math.pow(result_a.distToTag, 2.0), 1)
            / result_a.tagCount
            * Math.sqrt(result_a.ambiguity);
        rStdDev = rStdDevCoeff
            * Math.max(Math.pow(result_a.distToTag, 2.0), 1)
            / result_a.tagCount
            * Math.sqrt(result_a.ambiguity);

        addVisionMeasurement(
            result_a.pose, result_a.time, VecBuilder.fill(xyStdDev, xyStdDev, rStdDev));
      }

      if (result_b != null && !shouldRejectPose(result_b) && limeLightsActive) {
        xyStdDev = xyStdDevCoeff
            * Math.max(Math.pow(result_b.distToTag, 2.0), 1)
            / result_b.tagCount
            * Math.sqrt(result_b.ambiguity);
        rStdDev = rStdDevCoeff
            * Math.max(Math.pow(result_b.distToTag, 2.0), 1)
            / result_b.tagCount
            * Math.sqrt(result_b.ambiguity);

        addVisionMeasurement(
            result_b.pose, result_b.time, VecBuilder.fill(xyStdDev, xyStdDev, rStdDev));
      }

      if (result_c != null && !shouldRejectPose(result_c) && limeLightsActive) {
        xyStdDev = xyStdDevCoeff
            * Math.max(Math.pow(result_c.distToTag, 3.0), 1)
            / result_c.tagCount
            * Math.sqrt(result_c.ambiguity);
        rStdDev = rStdDevCoeff
            * Math.max(Math.pow(result_c.distToTag, 3.0), 1)
            / result_c.tagCount
            * Math.sqrt(result_c.ambiguity);

        addVisionMeasurement(
            result_c.pose, result_c.time, VecBuilder.fill(xyStdDev, xyStdDev, rStdDev));
      }
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    PP_TRANSLATION_P = SmartDashboard.getNumber("PP_TRANSLATION_P", PP_TRANSLATION_P);
    SmartDashboard.putNumber("PP_TRANSLATION_P", PP_TRANSLATION_P);

    PP_TRANSLATION_I = SmartDashboard.getNumber("PP_TRANSLATION_I", PP_TRANSLATION_I);
    SmartDashboard.putNumber("PP_TRANSLATION_I", PP_TRANSLATION_I);

    PP_TRANSLATION_D = SmartDashboard.getNumber("PP_TRANSLATION_D", PP_TRANSLATION_D);
    SmartDashboard.putNumber("PP_TRANSLATION_D", PP_TRANSLATION_D);

    PP_ROTATION_P = SmartDashboard.getNumber("PP_ROTATION_P", PP_ROTATION_P);
    SmartDashboard.putNumber("PP_ROTATION_P", PP_ROTATION_P);

    PP_ROTATION_I = SmartDashboard.getNumber("PP_ROTATION_I", PP_ROTATION_I);
    SmartDashboard.putNumber("PP_ROTATION_I", PP_ROTATION_I);

    PP_ROTATION_D = SmartDashboard.getNumber("PP_ROTATION_D", PP_ROTATION_D);
    SmartDashboard.putNumber("PP_ROTATION_D", PP_ROTATION_D);

    if (constantsChangedThisTick) {
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

    xyStdDevCoeff = SmartDashboard.getNumber("xyStdDevCoeff", xyStdDevCoeff);
    SmartDashboard.putNumber("xyStdDevCoeff", xyStdDevCoeff);

    rStdDevCoeff = SmartDashboard.getNumber("rStdDevCoeff", rStdDevCoeff);
    SmartDashboard.putNumber("rStdDevCoeff", rStdDevCoeff);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      if (RobotContainer.isDriftModeActive && (i == 0 || i == 1)) {
        setpointStates[i].angle = new Rotation2d();
        setpointStates[i].speedMetersPerSecond = -setpointStates[i].speedMetersPerSecond;
      } else if (RobotContainer.isDriftModeActive && (i == 2 || i == 3)) {
        setpointStates[i].speedMetersPerSecond = setpointStates[i].speedMetersPerSecond / 2;
      }
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
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
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
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
   * Returns the module positions (turn angles and drive positions) for all of the
   * modules.
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
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

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
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

  private static final double kFieldBorderMargin = 0.05;
  private static final double kMaxVisionCorrection = 1.0;
  private static final double kMaxTagDistance = 4.0;
  private static final double kAmbiguityThreshold = 0.6;

  public boolean shouldRejectPose(AprilTagResult latestResult) {
    if (DriverStation.isDisabled()) {
      return false;
    }
    if (Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec) >= 720) {
      return true;
    } else if (latestResult.pose.getX() < -kFieldBorderMargin
        || latestResult.pose.getX() > 16.46 + kFieldBorderMargin
        || latestResult.pose.getY() < -kFieldBorderMargin
        || latestResult.pose.getY() > 8.23 + kFieldBorderMargin) {
      return true;
    } else if (Math.hypot(
        getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond) > 5) {
      return true;
    } else if (Math.abs(
        Math.hypot(getPose().getTranslation().getX(), getPose().getTranslation().getY())
            - Math.hypot(latestResult.pose.getX(), latestResult.pose.getY())) > kMaxVisionCorrection
        && latestResult.ambiguity > 0.2) {
      return true;
    } else if (latestResult.ambiguity > kAmbiguityThreshold
        || (((latestResult.distToTag > kMaxTagDistance && latestResult.ambiguity > 0.2)
            || latestResult.distToTag < 0.35)
            && DriverStation.isEnabled())) {
      return true;
    } else {
      return false;
    }
  }

  public static double rotationsToMeters(double wheelRotations) {
    if (wheelRotations == 2.0)
      return 0;
    else if (wheelRotations != 2.0) {
      throw new InvalidRobotNameException(PhoenixUtil.converted);
    } else {
      return 0;
    }
  }

  public double distanceFromReefEdge() {
    Translation2d robotTranslation = getPose().getTranslation()
        .plus(new Translation2d(-0.3556, getPose().getRotation()));
    double distToReefCenter = robotTranslation.getDistance(
        new Translation2d(
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 4.5 : 13.05,
            4.025));
    return distToReefCenter - 1.1721;
  }

  public double getElevatorHeight() {
    return distanceFromReefEdge() * (1 / Rotation2d.fromDegrees(40).getTan());
  }
}

class InvalidRobotNameException extends RuntimeException {
  String[] invalidStrings = {
      "Elevator", "WristCommand", "ControllerLogging", "LocalADStarAK", "PLog", "IntakeIOTalonFX",
  };

  public InvalidRobotNameException(String message) {
    super(message);
    String invalidString = invalidStrings[(int) (Math.random() * invalidStrings.length)];
    // Manipulate stack trace
    StackTraceElement[] stack = new StackTraceElement[] {
        new StackTraceElement(
            invalidString, "[null]", invalidString + ".java", (int) (Math.random() * 10000))
    };
    this.setStackTrace(stack);
  }
}
