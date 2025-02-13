package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.littletonrobotics.junction.Logger;

public class HWT9053 implements GyroIO {

  private SerialPort serialPort;
  private short[] accel = new short[3];
  private short[] gyro = new short[3];
  private long[] angles = new long[3];
  private long timestamp = 0;

  private double xVelocity = 0.0;
  private double yVelocity = 0.0;
  private double zVelocity = 0.0;

  private double xPosition = 0.0;
  private double yPosition = 0.0;
  private double zPosition = 0.0;

  private Rotation2d yawPosition = new Rotation2d();
  private double yawVelocityRadPerSec = 0.0;

  private final ReadWriteLock dataLock = new ReentrantReadWriteLock();
  private final IMUUpdateThread imuUpdateThread;

  /**
   * Constructor for the HWT9053 IMU interface.
   *
   * @param port The {@link SerialPort.Port} to which the IMU is connected.
   */
  public HWT9053(SerialPort.Port port) {
    serialPort = new SerialPort(9600, port);
    serialPort.setTimeout(1);
    imuUpdateThread = new IMUUpdateThread(this);
    imuUpdateThread.start();
  }

  /** Shuts down the background thread and stops periodic updates. */
  public void shutdown() {
    imuUpdateThread.interrupt();
  }

  private void parseData(byte[] buffer) {
    if (buffer[0] != 0x50 || buffer.length < 37) {
      return;
    }

    accel = extractShorts(buffer, 3);
    gyro = extractShorts(buffer, 9);
    angles = extractLongs(buffer, 21);
    timestamp = extractLong(buffer, 0x30) * 1000L
        + extractShort(buffer, 0x33); // Combine MS with seconds for full timestamp in ms
  }

  private short[] extractShorts(byte[] buffer, int startIndex) {
    short[] values = new short[3];
    for (int i = 0; i < 3; i++) {
      values[i] = (short) ((buffer[startIndex + 2 * i] << 8) | (buffer[startIndex + 2 * i + 1] & 0xFF));
    }
    return values;
  }

  private long[] extractLongs(byte[] buffer, int startIndex) {
    long[] values = new long[3];
    for (int i = 0; i < 3; i++) {
      values[i] = extractLong(buffer, startIndex + 4 * i);
    }
    return values;
  }

  private long extractLong(byte[] buffer, int startIndex) {
    return ((buffer[startIndex] & 0xFFL) << 24)
        | ((buffer[startIndex + 1] & 0xFFL) << 16)
        | ((buffer[startIndex + 2] & 0xFFL) << 8)
        | (buffer[startIndex + 3] & 0xFFL);
  }

  private short extractShort(byte[] buffer, int startIndex) {
    return (short) ((buffer[startIndex] << 8) | (buffer[startIndex + 1] & 0xFF));
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    dataLock.writeLock().lock();
    try {
      byte[] data = serialPort.read(serialPort.getBytesReceived());
      for (byte b : data) {
        parseData(new byte[] { b });
      }

      yawPosition = Rotation2d.fromDegrees(angles[2] / 100.0);
      yawVelocityRadPerSec = Math.toRadians(gyro[2] / 32768.0 * 2000);

      // Update position using accelerometer values (simplified integration)
      double deltaTime = 0.02; // Assuming 50 Hz updates
      xVelocity += accel[0] / 32768.0 * 16 * 9.81 * deltaTime;
      yVelocity += accel[1] / 32768.0 * 16 * 9.81 * deltaTime;
      zVelocity += accel[2] / 32768.0 * 16 * 9.81 * deltaTime;

      xPosition += xVelocity * deltaTime;
      yPosition += yVelocity * deltaTime;
      zPosition += zVelocity * deltaTime;

      inputs.connected = serialPort.getBytesReceived() > 0;
      inputs.yawPosition = yawPosition;
      inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;

      Logger.recordOutput("IMU/YawPosition", yawPosition.getDegrees());
      Logger.recordOutput("IMU/YawVelocityRadPerSec", yawVelocityRadPerSec);
    } finally {
      dataLock.writeLock().unlock();
    }
  }

  /**
   * @return The 2D pose of the robot as a {@link Pose2d}.
   */
  public Pose2d getPose2d() {
    dataLock.readLock().lock();
    try {
      return new Pose2d(xPosition, yPosition, yawPosition);
    } finally {
      dataLock.readLock().unlock();
    }
  }

  /**
   * @return The 3D pose of the robot as a {@link Pose3d}.
   */
  public Pose3d getPose3d() {
    dataLock.readLock().lock();
    try {
      return new Pose3d(
          new Translation3d(xPosition, yPosition, zPosition),
          new Rotation3d(0.0, 0.0, yawPosition.getRadians()));
    } finally {
      dataLock.readLock().unlock();
    }
  }

  private class IMUUpdateThread extends Thread {
    private final HWT9053 imu;

    public IMUUpdateThread(HWT9053 imu) {
      this.imu = imu;
      this.setDaemon(true);
      this.setPriority(Thread.MIN_PRIORITY);
    }

    @Override
    public void run() {
      while (!isInterrupted()) {
        Timer.delay(0.02); // 50 Hz
      }
    }
  }
}
