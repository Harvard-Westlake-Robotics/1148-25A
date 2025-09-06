package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Advanced performance monitoring utility for optimizing robot periodic loop times. Tracks
 * execution times, identifies bottlenecks, and provides adaptive performance management.
 */
public class PerformanceMonitor {

  // Singleton instance
  private static PerformanceMonitor instance;

  // Performance tracking data
  private final Map<String, SubsystemPerformanceData> subsystemMetrics = new ConcurrentHashMap<>();
  private final Map<String, Double> currentLoopTimes = new HashMap<>();

  // Performance thresholds and settings
  private static final double TARGET_LOOP_TIME_MS = 20.0; // 50Hz target
  private static final double WARNING_THRESHOLD_MS = 18.0; // Warn at 90% of target
  private static final double CRITICAL_THRESHOLD_MS = 19.5; // Critical at 97.5% of target
  private static final double DEGRADED_MODE_THRESHOLD_MS = 22.0; // Enter degraded mode above target

  // Adaptive logging settings
  private static final int PERFORMANCE_HISTORY_SIZE = 50; // Track last 50 loops
  private static final double LOGGING_REDUCTION_FACTOR = 0.5; // Reduce logging by 50% when slow

  // Global performance state
  private boolean degradedModeActive = false;
  private double globalLoopOverhead = 0.0;
  private int consecutiveSlowLoops = 0;
  private double averageLoopTime = TARGET_LOOP_TIME_MS;

  private PerformanceMonitor() {
    // Private constructor for singleton
  }

  /**
   * Gets the singleton instance of PerformanceMonitor.
   *
   * @return The performance monitor instance
   */
  public static synchronized PerformanceMonitor getInstance() {
    if (instance == null) {
      instance = new PerformanceMonitor();
    }
    return instance;
  }

  /**
   * Starts timing a subsystem's periodic execution.
   *
   * @param subsystemName The name of the subsystem
   * @return The start timestamp for use with endTiming()
   */
  public double startTiming(String subsystemName) {
    double startTime = Timer.getFPGATimestamp();
    currentLoopTimes.put(subsystemName, startTime);
    return startTime;
  }

  /**
   * Ends timing for a subsystem and records the performance data.
   *
   * @param subsystemName The name of the subsystem
   * @param startTime The start timestamp from startTiming()
   */
  public void endTiming(String subsystemName, double startTime) {
    double endTime = Timer.getFPGATimestamp();
    double executionTime = (endTime - startTime) * 1000.0; // Convert to milliseconds

    recordPerformanceData(subsystemName, executionTime);
    currentLoopTimes.remove(subsystemName);
  }

  /**
   * Records performance data for a subsystem.
   *
   * @param subsystemName The name of the subsystem
   * @param executionTimeMs The execution time in milliseconds
   */
  private void recordPerformanceData(String subsystemName, double executionTimeMs) {
    SubsystemPerformanceData data =
        subsystemMetrics.computeIfAbsent(subsystemName, k -> new SubsystemPerformanceData());

    data.addSample(executionTimeMs);

    // Check for performance warnings
    checkPerformanceThresholds(subsystemName, executionTimeMs, data);

    // Update global performance metrics
    updateGlobalMetrics();
  }

  /**
   * Checks if execution time exceeds performance thresholds and logs warnings.
   *
   * @param subsystemName The name of the subsystem
   * @param executionTimeMs The execution time in milliseconds
   * @param data The performance data for the subsystem
   */
  private void checkPerformanceThresholds(
      String subsystemName, double executionTimeMs, SubsystemPerformanceData data) {
    if (executionTimeMs > CRITICAL_THRESHOLD_MS) {
      LoggingUtil.logError(
          subsystemName,
          "CRITICAL",
          String.format(
              "Periodic loop time %.2fms exceeds critical threshold (%.2fms)",
              executionTimeMs, CRITICAL_THRESHOLD_MS));
      consecutiveSlowLoops++;
    } else if (executionTimeMs > WARNING_THRESHOLD_MS) {
      LoggingUtil.logError(
          subsystemName,
          "WARNING",
          String.format(
              "Periodic loop time %.2fms exceeds warning threshold (%.2fms)",
              executionTimeMs, WARNING_THRESHOLD_MS));
      consecutiveSlowLoops++;
    } else {
      consecutiveSlowLoops = 0;
    }

    // Log performance metrics (reduced frequency in degraded mode)
    if (shouldLogPerformanceData()) {
      LoggingUtil.logDouble("Performance/" + subsystemName + "/ExecutionTimeMs", executionTimeMs);
      LoggingUtil.logDouble(
          "Performance/" + subsystemName + "/AverageTimeMs", data.getAverageTime());
      LoggingUtil.logDouble("Performance/" + subsystemName + "/MaxTimeMs", data.getMaxTime());
      LoggingUtil.logBoolean(
          "Performance/" + subsystemName + "/ExceedsThreshold",
          executionTimeMs > WARNING_THRESHOLD_MS);
    }
  }

  /** Updates global performance metrics and manages degraded mode. */
  private void updateGlobalMetrics() {
    // Calculate total execution time across all subsystems
    double totalExecutionTime =
        subsystemMetrics.values().stream()
            .mapToDouble(SubsystemPerformanceData::getLatestTime)
            .sum();

    averageLoopTime =
        subsystemMetrics.values().stream()
            .mapToDouble(SubsystemPerformanceData::getAverageTime)
            .sum();

    // Check if we should enter/exit degraded mode
    boolean shouldEnterDegradedMode =
        averageLoopTime > DEGRADED_MODE_THRESHOLD_MS || consecutiveSlowLoops > 5;

    if (shouldEnterDegradedMode && !degradedModeActive) {
      enterDegradedMode();
    } else if (!shouldEnterDegradedMode && degradedModeActive) {
      exitDegradedMode();
    }

    // Log global metrics
    if (shouldLogPerformanceData()) {
      LoggingUtil.logDouble("Performance/Global/TotalExecutionTimeMs", totalExecutionTime);
      LoggingUtil.logDouble("Performance/Global/AverageLoopTimeMs", averageLoopTime);
      LoggingUtil.logBoolean("Performance/Global/DegradedMode", degradedModeActive);
      LoggingUtil.logDouble("Performance/Global/ConsecutiveSlowLoops", consecutiveSlowLoops);
      LoggingUtil.logDouble("Performance/Global/TargetLoopTimeMs", TARGET_LOOP_TIME_MS);
    }
  }

  /** Enters degraded performance mode to reduce computational load. */
  private void enterDegradedMode() {
    degradedModeActive = true;
    LoggingUtil.logError(
        "PerformanceMonitor",
        "WARNING",
        "Entering degraded mode due to slow periodic loops. Reducing logging frequency and non-critical operations.");
  }

  /** Exits degraded performance mode when performance improves. */
  private void exitDegradedMode() {
    degradedModeActive = false;
    LoggingUtil.logError(
        "PerformanceMonitor",
        "INFO",
        "Exiting degraded mode. Performance has improved to acceptable levels.");
  }

  /**
   * Determines if performance data should be logged based on current mode.
   *
   * @return True if data should be logged, false otherwise
   */
  private boolean shouldLogPerformanceData() {
    if (!degradedModeActive) {
      return true;
    }

    // In degraded mode, log less frequently
    return Timer.getFPGATimestamp() % (TARGET_LOOP_TIME_MS / 1000.0 * 4)
        < 0.001; // Log every 4th loop
  }

  /**
   * Gets whether the system is currently in degraded performance mode.
   *
   * @return True if in degraded mode, false otherwise
   */
  public boolean isDegradedModeActive() {
    return degradedModeActive;
  }

  /**
   * Gets the current average loop time across all subsystems.
   *
   * @return Average loop time in milliseconds
   */
  public double getAverageLoopTime() {
    return averageLoopTime;
  }

  /**
   * Gets performance data for a specific subsystem.
   *
   * @param subsystemName The name of the subsystem
   * @return Performance data, or null if no data available
   */
  public SubsystemPerformanceData getSubsystemData(String subsystemName) {
    return subsystemMetrics.get(subsystemName);
  }

  /**
   * Gets the adaptive logging frequency multiplier based on performance.
   *
   * @return Multiplier for logging frequency (1.0 = normal, < 1.0 = reduced)
   */
  public double getLoggingFrequencyMultiplier() {
    if (degradedModeActive) {
      return LOGGING_REDUCTION_FACTOR;
    }
    return 1.0;
  }

  /**
   * Determines if a subsystem should skip non-critical operations to improve performance.
   *
   * @param subsystemName The name of the subsystem
   * @return True if non-critical operations should be skipped
   */
  public boolean shouldSkipNonCriticalOperations(String subsystemName) {
    SubsystemPerformanceData data = subsystemMetrics.get(subsystemName);
    if (data == null) return false;

    return degradedModeActive || data.getAverageTime() > WARNING_THRESHOLD_MS;
  }

  /**
   * Gets the recommended update frequency for a subsystem based on its performance.
   *
   * @param subsystemName The name of the subsystem
   * @param defaultFrequencyHz The default update frequency
   * @return Recommended frequency in Hz
   */
  public double getRecommendedUpdateFrequency(String subsystemName, double defaultFrequencyHz) {
    SubsystemPerformanceData data = subsystemMetrics.get(subsystemName);
    if (data == null) return defaultFrequencyHz;

    double averageTime = data.getAverageTime();

    if (averageTime > CRITICAL_THRESHOLD_MS) {
      return defaultFrequencyHz * 0.5; // Reduce to 50% frequency
    } else if (averageTime > WARNING_THRESHOLD_MS) {
      return defaultFrequencyHz * 0.75; // Reduce to 75% frequency
    }

    return defaultFrequencyHz;
  }

  /** Resets performance tracking data for all subsystems. */
  public void resetPerformanceData() {
    subsystemMetrics.clear();
    currentLoopTimes.clear();
    degradedModeActive = false;
    consecutiveSlowLoops = 0;
    averageLoopTime = TARGET_LOOP_TIME_MS;
  }

  /** Data class for tracking subsystem performance metrics. */
  public static class SubsystemPerformanceData {
    private final double[] executionTimes = new double[PERFORMANCE_HISTORY_SIZE];
    private int sampleCount = 0;
    private int currentIndex = 0;
    private double maxTime = 0.0;
    private double latestTime = 0.0;

    /**
     * Adds a new execution time sample.
     *
     * @param executionTimeMs The execution time in milliseconds
     */
    public void addSample(double executionTimeMs) {
      executionTimes[currentIndex] = executionTimeMs;
      currentIndex = (currentIndex + 1) % PERFORMANCE_HISTORY_SIZE;

      if (sampleCount < PERFORMANCE_HISTORY_SIZE) {
        sampleCount++;
      }

      maxTime = Math.max(maxTime, executionTimeMs);
      latestTime = executionTimeMs;
    }

    /**
     * Gets the average execution time over the tracking period.
     *
     * @return Average execution time in milliseconds
     */
    public double getAverageTime() {
      if (sampleCount == 0) return 0.0;

      double sum = 0.0;
      for (int i = 0; i < sampleCount; i++) {
        sum += executionTimes[i];
      }
      return sum / sampleCount;
    }

    /**
     * Gets the maximum execution time recorded.
     *
     * @return Maximum execution time in milliseconds
     */
    public double getMaxTime() {
      return maxTime;
    }

    /**
     * Gets the most recent execution time.
     *
     * @return Latest execution time in milliseconds
     */
    public double getLatestTime() {
      return latestTime;
    }

    /**
     * Gets the number of samples collected.
     *
     * @return Sample count
     */
    public int getSampleCount() {
      return sampleCount;
    }
  }
}
