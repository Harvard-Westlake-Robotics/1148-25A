package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.util.LoggingUtil;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final WristIOTalonFX io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private WristConstants constants;
  private String key;
  private static Climb instance;
  private DigitalInput limitSwitch = new DigitalInput(0);

  // Tunable parameters
  private double targetAngle = 90 * 5; // Default position
  private double angleTolerance = 5.0; // degrees

  public static Climb getInstance() {
    if (instance == null) {
      instance = new Climb();
    }
    return instance;
  }

  public Climb() {
    this.constants = Constants.HangWrist;
    this.key = "Hang Wrist";
    io = new WristIOTalonFX(constants);
    io.zeroPosition(90 * 5);
    io.setAngle(90 * 5);
  }

  public WristConstants getConstants() {
    return constants;
  }

  public void periodic() {
    double startTime = Timer.getFPGATimestamp();

    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);

    // Log comprehensive climb state
    logClimbState();

    // Update tunable parameters
    updateTunableParameters();

    // Log performance metrics
    double endTime = Timer.getFPGATimestamp();
    LoggingUtil.logPerformanceMetrics("Climb", endTime - startTime, 50.0); // Target 50Hz
  }

  public void goToAngle(double angle) {
    this.targetAngle = angle;
    this.io.setAngle(angle);
    LoggingUtil.logDouble("Climb/TargetAngle", angle);
  }

  public void runVoltage(double volts) {
    io.runCharacterization(volts);
  }

  public double getWristPosition() {
    return inputs.wristPositionMeters / constants.motorToWristRotations;
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  /** Logs comprehensive climb state information */
  private void logClimbState() {
    // Basic climb state
    LoggingUtil.logSubsystemStatus("Climb", true, "NORMAL_MODE");

    // Position and movement
    double currentAngle = getWristPosition();
    LoggingUtil.logDouble("Climb/Position", currentAngle);
    LoggingUtil.logDouble("Climb/TargetAngle", targetAngle);
    LoggingUtil.logDouble("Climb/AngleError", Math.abs(targetAngle - currentAngle));
    LoggingUtil.logBoolean("Climb/AtTarget", Math.abs(targetAngle - currentAngle) < angleTolerance);

    // Motor data
    LoggingUtil.logMotorData(
        "Climb",
        "Motor",
        inputs.wristAppliedVolts,
        inputs.wristCurrentAmps,
        0.0, // No temperature sensor available
        inputs.wristPositionMeters,
        inputs.wristVelocityMPS);

    // Limit switch and safety
    LoggingUtil.logBooleanSensor("Climb", "LimitSwitch", getLimitSwitch());

    // Power consumption
    LoggingUtil.logDouble(
        "Climb/PowerConsumption", inputs.wristAppliedVolts * inputs.wristCurrentAmps);
  }

  /** Updates all tunable parameters from SmartDashboard */
  private void updateTunableParameters() {
    angleTolerance = LoggingUtil.getTunableDouble("Climb/AngleTolerance", angleTolerance);

    // Log current tunable values
    LoggingUtil.logDouble("Climb/Tuning/AngleTolerance", angleTolerance);
  }

  public double getAngleError() {
    return Math.abs(targetAngle - getWristPosition());
  }

  public boolean isAtTarget() {
    return Math.abs(targetAngle - getWristPosition()) < angleTolerance;
  }

  public double getTargetAngle() {
    return targetAngle;
  }
}
