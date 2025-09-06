package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utility class for managing performance optimization settings and providing system-wide
 * performance recommendations.
 */
public class PerformanceOptimizer {

  // Performance optimization settings
  public static final class Settings {
    // Global performance targets
    public static final double TARGET_LOOP_TIME_MS = 20.0;
    public static final double WARNING_THRESHOLD_PERCENT = 90.0;
    public static final double CRITICAL_THRESHOLD_PERCENT = 97.5;

    // Adaptive frequency settings
    public static final double VISION_DEGRADED_FREQUENCY_HZ = 12.5;
    public static final double LOGGING_DEGRADED_FREQUENCY_HZ = 5.0;
    public static final double DASHBOARD_DEGRADED_FREQUENCY_HZ = 2.0;

    // Performance improvement factors
    public static final double LOGGING_REDUCTION_FACTOR = 0.5;
    public static final double NON_CRITICAL_SKIP_FACTOR = 0.25;

    // Subsystem priority levels (higher = more critical)
    public static final int PRIORITY_CRITICAL = 10; // Safety systems
    public static final int PRIORITY_HIGH = 8; // Drive, odometry
    public static final int PRIORITY_MEDIUM = 5; // Mechanisms
    public static final int PRIORITY_LOW = 2; // Logging, dashboard

    private Settings() {} // Utility class
  }

  /**
   * Gets the recommended update frequency for a subsystem based on performance.
   *
   * @param subsystemName The name of the subsystem
   * @param baseFrequencyHz The normal operating frequency
   * @param priority The priority level of the subsystem
   * @return Recommended frequency in Hz
   */
  public static double getRecommendedFrequency(
      String subsystemName, double baseFrequencyHz, int priority) {
    PerformanceMonitor monitor = PerformanceMonitor.getInstance();

    if (!monitor.isDegradedModeActive()) {
      return baseFrequencyHz;
    }

    // Apply frequency reduction based on priority
    double reductionFactor = calculateReductionFactor(priority);
    return baseFrequencyHz * reductionFactor;
  }

  /**
   * Calculates the reduction factor based on subsystem priority.
   *
   * @param priority The priority level of the subsystem
   * @return Reduction factor (0.0 to 1.0)
   */
  private static double calculateReductionFactor(int priority) {
    if (priority >= Settings.PRIORITY_CRITICAL) {
      return 1.0; // No reduction for critical systems
    } else if (priority >= Settings.PRIORITY_HIGH) {
      return 0.8; // 20% reduction for high priority
    } else if (priority >= Settings.PRIORITY_MEDIUM) {
      return 0.5; // 50% reduction for medium priority
    } else {
      return 0.25; // 75% reduction for low priority
    }
  }

  /**
   * Determines if a subsystem should skip non-essential operations.
   *
   * @param subsystemName The name of the subsystem
   * @param priority The priority level of the subsystem
   * @return True if non-essential operations should be skipped
   */
  public static boolean shouldSkipNonEssentialOperations(String subsystemName, int priority) {
    PerformanceMonitor monitor = PerformanceMonitor.getInstance();

    // Critical systems never skip operations
    if (priority >= Settings.PRIORITY_CRITICAL) {
      return false;
    }

    // Use performance monitor's recommendation for other systems
    return monitor.shouldSkipNonCriticalOperations(subsystemName);
  }

  /**
   * Gets performance optimization recommendations for dashboard display.
   *
   * @return Performance status and recommendations
   */
  public static PerformanceStatus getPerformanceStatus() {
    PerformanceMonitor monitor = PerformanceMonitor.getInstance();

    double averageLoopTime = monitor.getAverageLoopTime();
    boolean degradedMode = monitor.isDegradedModeActive();

    String status;
    String recommendation;

    if (averageLoopTime
        <= Settings.TARGET_LOOP_TIME_MS * Settings.WARNING_THRESHOLD_PERCENT / 100.0) {
      status = "OPTIMAL";
      recommendation = "Performance is good. All systems operating normally.";
    } else if (averageLoopTime
        <= Settings.TARGET_LOOP_TIME_MS * Settings.CRITICAL_THRESHOLD_PERCENT / 100.0) {
      status = "WARNING";
      recommendation = "Performance approaching limits. Consider reducing logging frequency.";
    } else if (!degradedMode) {
      status = "CRITICAL";
      recommendation = "Performance critical. Enabling degraded mode to maintain loop timing.";
    } else {
      status = "DEGRADED";
      recommendation = "Operating in degraded mode. Non-critical operations reduced.";
    }

    return new PerformanceStatus(status, recommendation, averageLoopTime, degradedMode);
  }

  /**
   * Updates performance optimization settings from SmartDashboard. Allows for real-time tuning of
   * performance parameters.
   */
  public static void updateSettingsFromDashboard() {
    // These could be made tunable if needed for testing
    SmartDashboard.putNumber("Performance/TargetLoopTimeMs", Settings.TARGET_LOOP_TIME_MS);
    SmartDashboard.putNumber(
        "Performance/WarningThresholdPercent", Settings.WARNING_THRESHOLD_PERCENT);
    SmartDashboard.putNumber(
        "Performance/CriticalThresholdPercent", Settings.CRITICAL_THRESHOLD_PERCENT);

    PerformanceStatus status = getPerformanceStatus();
    SmartDashboard.putString("Performance/Status", status.status);
    SmartDashboard.putString("Performance/Recommendation", status.recommendation);
    SmartDashboard.putNumber("Performance/AverageLoopTimeMs", status.averageLoopTime);
    SmartDashboard.putBoolean("Performance/DegradedMode", status.degradedMode);
  }

  /** Data class for holding performance status information. */
  public static class PerformanceStatus {
    public final String status;
    public final String recommendation;
    public final double averageLoopTime;
    public final boolean degradedMode;

    public PerformanceStatus(
        String status, String recommendation, double averageLoopTime, boolean degradedMode) {
      this.status = status;
      this.recommendation = recommendation;
      this.averageLoopTime = averageLoopTime;
      this.degradedMode = degradedMode;
    }
  }
}
