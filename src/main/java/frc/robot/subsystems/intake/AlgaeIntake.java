package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LoggingUtil;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase {
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeConstants constants;
  private String key;
  private static AlgaeIntake instance;
  private Boolean hasAlgae = false;

  // Tunable parameters
  private double velocityTolerance = 0.1; // m/s
  private double currentThreshold = 15.0; // amps
  private LinearVelocity targetVelocity;

  public Boolean getHasAlgae() {
    return hasAlgae;
  }

  public void setHasAlgae(Boolean hasAlgae) {
    this.hasAlgae = hasAlgae;
  }

  public static AlgaeIntake getInstance() {
    if (instance == null) {
      instance = new AlgaeIntake();
    }
    return instance;
  }

  public AlgaeIntake() {
    this.constants = Constants.AlgaeIntake;
    this.key = "Algae Intake";
    io = new IntakeIOTalonFX(constants);
  }

  public IntakeConstants getConstants() {
    return constants;
  }

  public void periodic() {
    double startTime = Timer.getFPGATimestamp();

    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);

    // Update algae detection logic
    hasAlgae = inputs.intakeAppliedVolts > 1.0 && inputs.intakeVelocityMPS < 0.2;

    // Log comprehensive intake state
    logIntakeState();

    // Update tunable parameters
    updateTunableParameters();

    // Log performance metrics
    double endTime = Timer.getFPGATimestamp();
    LoggingUtil.logPerformanceMetrics("AlgaeIntake", endTime - startTime, 50.0); // Target 50Hz
  }

  public void setVelocity(LinearVelocity velocity) {
    this.targetVelocity = velocity;
    io.runVelocity(velocity);
    LoggingUtil.logDouble("AlgaeIntake/TargetVelocity", velocity.magnitude());
  }

  public void push(double rotations) {
    io.push(rotations);
  }

  public void runVoltage(double volts) {
    io.runCharacterization(volts);
  }

  public Boolean getSensor1() {
    return io.getSensor1();
  }

  public Boolean getSensor2() {
    return io.getSensor2();
  }

  /** Logs comprehensive algae intake state information */
  private void logIntakeState() {
    // Basic intake state
    LoggingUtil.logSubsystemStatus("AlgaeIntake", true, "NORMAL_MODE");

    // Velocity and control
    LoggingUtil.logDouble("AlgaeIntake/Velocity", inputs.intakeVelocityMPS);
    if (targetVelocity != null) {
      LoggingUtil.logDouble("AlgaeIntake/TargetVelocity", targetVelocity.magnitude());
      LoggingUtil.logDouble(
          "AlgaeIntake/VelocityError",
          Math.abs(targetVelocity.magnitude() - inputs.intakeVelocityMPS));
      LoggingUtil.logBoolean(
          "AlgaeIntake/AtTargetVelocity",
          Math.abs(targetVelocity.magnitude() - inputs.intakeVelocityMPS) < velocityTolerance);
    }

    // Motor data
    LoggingUtil.logMotorData(
        "AlgaeIntake",
        "Motor",
        inputs.intakeAppliedVolts,
        inputs.intakeCurrentAmps,
        0.0, // No temperature sensor available
        inputs.intakePositionMeters,
        inputs.intakeVelocityMPS);

    // Sensor data
    LoggingUtil.logBooleanSensor(
        "AlgaeIntake", "Sensor1", getSensor1() != null ? getSensor1() : false);
    LoggingUtil.logBooleanSensor(
        "AlgaeIntake", "Sensor2", getSensor2() != null ? getSensor2() : false);

    // Game piece detection
    LoggingUtil.logBoolean("AlgaeIntake/HasAlgae", hasAlgae);
    LoggingUtil.logBoolean("AlgaeIntake/HighCurrent", inputs.intakeCurrentAmps > currentThreshold);

    // Power consumption
    LoggingUtil.logDouble(
        "AlgaeIntake/PowerConsumption", inputs.intakeAppliedVolts * inputs.intakeCurrentAmps);
  }

  /** Updates all tunable parameters from SmartDashboard */
  private void updateTunableParameters() {
    velocityTolerance =
        LoggingUtil.getTunableDouble("AlgaeIntake/VelocityTolerance", velocityTolerance);
    currentThreshold =
        LoggingUtil.getTunableDouble("AlgaeIntake/CurrentThreshold", currentThreshold);

    // Log current tunable values
    LoggingUtil.logDouble("AlgaeIntake/Tuning/VelocityTolerance", velocityTolerance);
    LoggingUtil.logDouble("AlgaeIntake/Tuning/CurrentThreshold", currentThreshold);
  }

  public double getVelocityError() {
    if (targetVelocity == null) return 0.0;
    return Math.abs(targetVelocity.magnitude() - inputs.intakeVelocityMPS);
  }

  public boolean isAtTargetVelocity() {
    if (targetVelocity == null) return true;
    return Math.abs(targetVelocity.magnitude() - inputs.intakeVelocityMPS) < velocityTolerance;
  }
}
