package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class for consistent logging across all subsystems. Provides standardized methods for
 * logging telemetry data and managing tunable parameters.
 */
public class LoggingUtil {

  /**
   * Logs a double value with consistent formatting
   *
   * @param key The logging key (subsystem/parameter format)
   * @param value The value to log
   */
  public static void logDouble(String key, double value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Logs a boolean value with consistent formatting
   *
   * @param key The logging key (subsystem/parameter format)
   * @param value The value to log
   */
  public static void logBoolean(String key, boolean value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Logs a string value with consistent formatting
   *
   * @param key The logging key (subsystem/parameter format)
   * @param value The value to log
   */
  public static void logString(String key, String value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Logs an array of doubles with consistent formatting
   *
   * @param key The logging key (subsystem/parameter format)
   * @param values The array to log
   */
  public static void logDoubleArray(String key, double[] values) {
    Logger.recordOutput(key, values);
  }

  /**
   * Creates and manages a tunable double parameter via SmartDashboard
   *
   * @param key The dashboard key
   * @param defaultValue The default value
   * @return The current value from the dashboard
   */
  public static double getTunableDouble(String key, double defaultValue) {
    SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
    return SmartDashboard.getNumber(key, defaultValue);
  }

  /**
   * Creates and manages a tunable boolean parameter via SmartDashboard
   *
   * @param key The dashboard key
   * @param defaultValue The default value
   * @return The current value from the dashboard
   */
  public static boolean getTunableBoolean(String key, boolean defaultValue) {
    SmartDashboard.putBoolean(key, SmartDashboard.getBoolean(key, defaultValue));
    return SmartDashboard.getBoolean(key, defaultValue);
  }

  /**
   * Creates and manages a tunable string parameter via SmartDashboard
   *
   * @param key The dashboard key
   * @param defaultValue The default value
   * @return The current value from the dashboard
   */
  public static String getTunableString(String key, String defaultValue) {
    SmartDashboard.putString(key, SmartDashboard.getString(key, defaultValue));
    return SmartDashboard.getString(key, defaultValue);
  }

  /**
   * Logs subsystem status information in a consistent format
   *
   * @param subsystemName The name of the subsystem
   * @param enabled Whether the subsystem is enabled
   * @param status Current status message
   */
  public static void logSubsystemStatus(String subsystemName, boolean enabled, String status) {
    Logger.recordOutput(subsystemName + "/Enabled", enabled);
    Logger.recordOutput(subsystemName + "/Status", status);
    Logger.recordOutput(subsystemName + "/Timestamp", Logger.getRealTimestamp());
  }

  /**
   * Logs motor information in a consistent format
   *
   * @param subsystemName The name of the subsystem
   * @param motorName The name/ID of the motor
   * @param voltage Applied voltage
   * @param current Current draw
   * @param temperature Motor temperature
   * @param position Position/angle
   * @param velocity Velocity
   */
  public static void logMotorData(
      String subsystemName,
      String motorName,
      double voltage,
      double current,
      double temperature,
      double position,
      double velocity) {
    String prefix = subsystemName + "/Motors/" + motorName + "/";
    Logger.recordOutput(prefix + "Voltage", voltage);
    Logger.recordOutput(prefix + "Current", current);
    Logger.recordOutput(prefix + "Temperature", temperature);
    Logger.recordOutput(prefix + "Position", position);
    Logger.recordOutput(prefix + "Velocity", velocity);
  }

  /**
   * Logs PID controller data in a consistent format
   *
   * @param subsystemName The name of the subsystem
   * @param controllerName The name of the PID controller
   * @param setpoint Current setpoint
   * @param measurement Current measurement
   * @param error Current error
   * @param output Current output
   */
  public static void logPIDData(
      String subsystemName,
      String controllerName,
      double setpoint,
      double measurement,
      double error,
      double output) {
    String prefix = subsystemName + "/PID/" + controllerName + "/";
    Logger.recordOutput(prefix + "Setpoint", setpoint);
    Logger.recordOutput(prefix + "Measurement", measurement);
    Logger.recordOutput(prefix + "Error", error);
    Logger.recordOutput(prefix + "Output", output);
  }

  /**
   * Logs sensor data in a consistent format
   *
   * @param subsystemName The name of the subsystem
   * @param sensorName The name of the sensor
   * @param value The sensor value
   * @param units The units of measurement
   */
  public static void logSensorData(
      String subsystemName, String sensorName, double value, String units) {
    String prefix = subsystemName + "/Sensors/" + sensorName + "/";
    Logger.recordOutput(prefix + "Value", value);
    Logger.recordOutput(prefix + "Units", units);
  }

  /**
   * Logs boolean sensor data (limit switches, etc.)
   *
   * @param subsystemName The name of the subsystem
   * @param sensorName The name of the sensor
   * @param value The sensor state
   */
  public static void logBooleanSensor(String subsystemName, String sensorName, boolean value) {
    Logger.recordOutput(subsystemName + "/Sensors/" + sensorName, value);
  }

  /**
   * Logs performance metrics for subsystems
   *
   * @param subsystemName The name of the subsystem
   * @param loopTime The time taken for the periodic loop
   * @param targetFrequency The target frequency for the subsystem
   */
  public static void logPerformanceMetrics(
      String subsystemName, double loopTime, double targetFrequency) {
    String prefix = subsystemName + "/Performance/";
    Logger.recordOutput(prefix + "LoopTime", loopTime);
    Logger.recordOutput(prefix + "TargetFrequency", targetFrequency);
    Logger.recordOutput(prefix + "ActualFrequency", 1.0 / loopTime);
  }

  /**
   * Logs error conditions and warnings
   *
   * @param subsystemName The name of the subsystem
   * @param errorLevel The severity level (INFO, WARNING, ERROR, CRITICAL)
   * @param message The error message
   */
  public static void logError(String subsystemName, String errorLevel, String message) {
    String prefix = subsystemName + "/Errors/";
    Logger.recordOutput(prefix + "Level", errorLevel);
    Logger.recordOutput(prefix + "Message", message);
    Logger.recordOutput(prefix + "Timestamp", Logger.getRealTimestamp());

    // Also log to console for immediate visibility
    System.err.println("[" + errorLevel + "] " + subsystemName + ": " + message);
  }
}
