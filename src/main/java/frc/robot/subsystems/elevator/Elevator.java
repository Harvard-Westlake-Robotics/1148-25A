package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.util.LoggingUtil;
import frc.robot.util.PerformanceMonitor;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private static Elevator instance = null;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  public DigitalInput dio = new DigitalInput(5);
  SysIdRoutine sysId;

  // Tunable parameters
  private double targetHeight = 0.0;
  private double heightTolerance = 0.02; // meters
  private boolean isOverriding = false;

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  private Elevator() {
    io = new ElevatorIOTalonFX();
    sysId =
        new SysIdRoutine(
            new Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  public void periodic() {
    // Start performance monitoring
    PerformanceMonitor monitor = PerformanceMonitor.getInstance();
    double periodicStartTime = monitor.startTiming("Elevator");

    // Core sensor updates (always execute)
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Handle limit switch for auto-zeroing (safety critical)
    if (!dio.get() && inputs.elevator1PositionMeters >= 0.05) {
      io.zeroMotors();
      LoggingUtil.logString("Elevator/Status", "AUTO_ZEROED");
    }

    // Non-critical updates (skip if performance is poor)
    if (!monitor.shouldSkipNonCriticalOperations("Elevator")) {
      // Log comprehensive elevator state
      logElevatorState();

      // Update tunable parameters
      updateTunableParameters();
    } else {
      // In performance-critical mode, only log essential data
      logCriticalElevatorState();
    }

    // End performance monitoring
    monitor.endTiming("Elevator", periodicStartTime);
  }

  public void goToHeight(double height) {
    this.targetHeight = height;
    io.setHeightClosedLoop(height);
    LoggingUtil.logDouble("Elevator/TargetHeight", height);
  }

  public void goToHeightMeters(double height) {
    io.setHeightMetersAdjusted(height);
  }

  public double getHeight() {
    return inputs.elevator1PositionMeters;
  }

  public void goToHeightOverride(double height) {
    io.setHeightClosedLoopOverride(getHeight());
  }

  public void setOverride(boolean over) {
    this.isOverriding = over;
    io.setIsOverriding(over);
    LoggingUtil.logBoolean("Elevator/Override", over);
  }

  /** Logs comprehensive elevator state information */
  private void logElevatorState() {
    // Basic elevator state
    LoggingUtil.logSubsystemStatus(
        "Elevator", true, isOverriding ? "OVERRIDE_MODE" : "NORMAL_MODE");

    // Position and movement
    LoggingUtil.logDouble("Elevator/Position", inputs.elevator1PositionMeters);
    LoggingUtil.logDouble("Elevator/Velocity", inputs.elevator1VelocityMPS);
    LoggingUtil.logDouble("Elevator/TargetHeight", targetHeight);
    LoggingUtil.logDouble(
        "Elevator/PositionError", Math.abs(targetHeight - inputs.elevator1PositionMeters));
    LoggingUtil.logBoolean(
        "Elevator/AtTarget",
        Math.abs(targetHeight - inputs.elevator1PositionMeters) < heightTolerance);

    // Motor data for both elevators
    LoggingUtil.logMotorData(
        "Elevator",
        "Motor1",
        inputs.elevator1AppliedVolts,
        inputs.elevator1CurrentAmps,
        0.0, // No temperature sensor available
        inputs.elevator1PositionMeters,
        inputs.elevator1VelocityMPS);

    LoggingUtil.logMotorData(
        "Elevator",
        "Motor2",
        inputs.elevator2AppliedVolts,
        inputs.elevator2CurrentAmps,
        0.0, // No temperature sensor available
        inputs.elevator2PositionMeters,
        inputs.elevator2VelocityMPS);

    // Limit switch and safety
    LoggingUtil.logBooleanSensor("Elevator", "LimitSwitch", !dio.get());
    LoggingUtil.logBoolean("Elevator/SafeToMove", inputs.elevator1PositionMeters >= -0.1);

    // System state
    LoggingUtil.logBoolean("Elevator/Override", isOverriding);
    LoggingUtil.logDouble(
        "Elevator/PowerConsumption",
        inputs.elevator1AppliedVolts * inputs.elevator1CurrentAmps
            + inputs.elevator2AppliedVolts * inputs.elevator2CurrentAmps);
  }

  /** Updates all tunable parameters from SmartDashboard */
  private void updateTunableParameters() {
    heightTolerance = LoggingUtil.getTunableDouble("Elevator/HeightTolerance", heightTolerance);

    // Log current tunable values
    LoggingUtil.logDouble("Elevator/Tuning/HeightTolerance", heightTolerance);
  }

  /**
   * Logs only critical elevator state when performance is limited. Reduces logging overhead while
   * maintaining essential telemetry.
   */
  private void logCriticalElevatorState() {
    // Only log essential status and position data
    LoggingUtil.logSubsystemStatus("Elevator", true, isOverriding ? "OVERRIDE" : "NORMAL");
    LoggingUtil.logDouble("Elevator/Position", inputs.elevator1PositionMeters);
    LoggingUtil.logDouble("Elevator/TargetHeight", targetHeight);
    LoggingUtil.logBoolean(
        "Elevator/AtTarget",
        Math.abs(inputs.elevator1PositionMeters - targetHeight) < heightTolerance);
  }

  public double getTargetHeight() {
    return targetHeight;
  }

  public boolean isAtTarget() {
    return Math.abs(targetHeight - inputs.elevator1PositionMeters) < heightTolerance;
  }

  public double getPositionError() {
    return Math.abs(targetHeight - inputs.elevator1PositionMeters);
  }

  public void runCharacterization(double voltage) {
    io.runCharacterization(voltage);
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

  public double getTarget() {
    return io.getTarget();
  }
}
