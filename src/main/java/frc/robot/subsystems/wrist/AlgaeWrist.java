package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.util.LoggingUtil;
import org.littletonrobotics.junction.Logger;

public class AlgaeWrist extends SubsystemBase {
  private final WristIOTalonFX io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private WristConstants constants;
  private String key;
  private static AlgaeWrist instance;

  // Tunable parameters
  private double targetAngle = 0.0;
  private double angleTolerance = 2.0; // degrees

  public static AlgaeWrist getInstance() {
    if (instance == null) {
      instance = new AlgaeWrist();
    }
    return instance;
  }

  public AlgaeWrist() {
    this.constants = Constants.IntakeWrist;
    this.key = "Intake Wrist";
    io = new WristIOTalonFX(constants);
  }

  public WristConstants getConstants() {
    return constants;
  }

  public void periodic() {
    double startTime = Timer.getFPGATimestamp();

    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);

    // Log comprehensive wrist state
    logWristState();

    // Update tunable parameters
    updateTunableParameters();

    // Log performance metrics
    double endTime = Timer.getFPGATimestamp();
    LoggingUtil.logPerformanceMetrics("AlgaeWrist", endTime - startTime, 50.0); // Target 50Hz
  }

  public void goToAngle(double angle) {
    this.targetAngle = angle;
    this.io.setAngle(angle);
    LoggingUtil.logDouble("AlgaeWrist/TargetAngle", angle);
  }

  public void runVoltage(double volts) {
    io.runCharacterization(volts);
  }

  public double getWristPosition() {
    return inputs.wristPositionMeters / constants.motorToWristRotations;
  }

  /** Logs comprehensive algae wrist state information */
  private void logWristState() {
    // Basic wrist state
    LoggingUtil.logSubsystemStatus("AlgaeWrist", true, "NORMAL_MODE");

    // Position and movement
    double currentAngle = getWristPosition();
    LoggingUtil.logDouble("AlgaeWrist/Position", currentAngle);
    LoggingUtil.logDouble("AlgaeWrist/TargetAngle", targetAngle);
    LoggingUtil.logDouble("AlgaeWrist/AngleError", Math.abs(targetAngle - currentAngle));
    LoggingUtil.logBoolean(
        "AlgaeWrist/AtTarget", Math.abs(targetAngle - currentAngle) < angleTolerance);

    // Motor data
    LoggingUtil.logMotorData(
        "AlgaeWrist",
        "Motor",
        inputs.wristAppliedVolts,
        inputs.wristCurrentAmps,
        0.0, // No temperature sensor available
        inputs.wristPositionMeters,
        inputs.wristVelocityMPS);

    // Power consumption
    LoggingUtil.logDouble(
        "AlgaeWrist/PowerConsumption", inputs.wristAppliedVolts * inputs.wristCurrentAmps);
  }

  /** Updates all tunable parameters from SmartDashboard */
  private void updateTunableParameters() {
    angleTolerance = LoggingUtil.getTunableDouble("AlgaeWrist/AngleTolerance", angleTolerance);

    // Log current tunable values
    LoggingUtil.logDouble("AlgaeWrist/Tuning/AngleTolerance", angleTolerance);
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
