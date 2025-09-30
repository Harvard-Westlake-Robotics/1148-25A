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

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class IntakeConstants {
    public final int motorId;
    public final InvertedValue motorInverted;
    public final double intakeVelocity;
    public final double outtakeVelocity;
    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;
    public double kA;
    public double positionkP;
    public double positionkD;
    public int sensor1ID;
    public int sensor2ID;
    public int sensor3ID;

    public final double ANGLE_MAX_ACCELERATION;
    public final double ANGLE_MAX_VELOCITY;
    public final double ANGLe_MAX_JERK;
    public final double rotationsToMetersRatio;

    public IntakeConstants(
        int motorId,
        InvertedValue motorInverted,
        double intakeVelocity,
        double outtakeVelocity,
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA,
        int sensor1ID,
        int sensor2ID,
        int sensor3ID,
        double positionkP,
        double positionkD,
        double ANGLE_MAX_ACCELERATION,
        double ANGLE_MAX_VELOCITY,
        double ANGLe_MAX_JERK,
        double rotationsToMetersRatio) {
      this.motorId = motorId;
      this.motorInverted = motorInverted;
      this.intakeVelocity = intakeVelocity;
      this.outtakeVelocity = outtakeVelocity;
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
      this.sensor1ID = sensor1ID;
      this.sensor2ID = sensor2ID;
      this.sensor3ID = sensor3ID;
      this.positionkP = positionkP;
      this.positionkD = positionkD;
      this.ANGLE_MAX_ACCELERATION = ANGLE_MAX_ACCELERATION;
      this.ANGLE_MAX_VELOCITY = ANGLE_MAX_VELOCITY;
      this.ANGLe_MAX_JERK = ANGLe_MAX_JERK;
      this.rotationsToMetersRatio = rotationsToMetersRatio;
    }
  }

  /*
   * in the parameters below, we did the reciprocal of rotationsToMetersRatio bc
   * your dumbass did metersToRotations :)
   */
  public static final IntakeConstants AlgaeIntake =
      new IntakeConstants(
          16,
          InvertedValue.CounterClockwise_Positive,
          100,
          -100,
          1.669162,
          0.0,
          0.0,
          0.1761,
          0.12875,
          0.0,
          -1,
          -1,
          -1,
          0.0,
          0.0,
          100.0,
          100.0,
          1000000.0,
          1.0 / 16.709);

  public static final IntakeConstants CoralIntake =
      new IntakeConstants(
          17,
          InvertedValue.Clockwise_Positive,
          100,
          -100,
          8.569162,
          0.0,
          0.0,
          0.3,
          0.0,
          0.0,
          4,
          7,
          6,
          0.4,
          0.000,
          99999.0,
          99999.0,
          99999.0,
          1.0); // / 16.709

  public final class Elevator {
    public static final int elevator1ID = 13;
    public static final int elevator2ID = 14;
    public static final InvertedValue elevator2Inverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue elevator1Inverted = InvertedValue.Clockwise_Positive;
    public static double kP = 1.75;
    public static double kI = 0.01;
    public static double kD = 0.00;
    public static double kS = 0.1;
    public static double kV = 0.0;
    public static double kG = 0.0;
    public static double kA = 0.0;
    public static final double elevatorForwardSoftLimitRotations = 55;
    public static final double elevatorReverseSoftLimitRotations = 0.0;
    public static final double rotationsToMetersRatio = (24.02);
    public static final double[] elevatorHeights = {0, 1, 2, 3};
    public static final double elevatorGroundOffsetMeters = 0.2125;
  }

  public static class WristConstants {
    public final int motorId;
    public final InvertedValue motorInverted;
    public final double wristVelocity;
    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;
    public double kG;
    public double kA;

    public double ANGLE_MAX_ACCELERATION;
    public double ANGLE_MAX_VELOCITY;
    public double ANGLe_MAX_JERK;
    public double motorToWristRotations;
    public Angle angleOffset;
    public int statorLimit;
    public int supplyLimit;

    public WristConstants(
        int motorId,
        InvertedValue motorInverted,
        double wristVelocity,
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kG,
        double kA,
        double ANGLE_MAX_ACCELERATION,
        double ANGLE_MAX_VELOCITY,
        double ANGLe_MAX_JERK,
        double motorToWristRotations,
        Angle angleOffset,
        int supplyLimit,
        int statorLimit) {
      this.motorId = motorId;
      this.motorInverted = motorInverted;
      this.wristVelocity = wristVelocity;
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kS = kS;
      this.kV = kV;
      this.kG = kG;
      this.kA = kA;
      this.ANGLE_MAX_ACCELERATION = ANGLE_MAX_ACCELERATION;
      this.ANGLE_MAX_VELOCITY = ANGLE_MAX_VELOCITY;
      this.ANGLe_MAX_JERK = ANGLe_MAX_JERK;
      this.motorToWristRotations = motorToWristRotations;
      this.angleOffset = angleOffset;
      this.supplyLimit = supplyLimit;
      this.statorLimit = statorLimit;
    }
  }

  // TODO: Fix later with real values
  public static final WristConstants IntakeWrist =
      new WristConstants(
          18,
          InvertedValue.CounterClockwise_Positive,
          100000,
          3.669162,
          0.0,
          0.2,
          0.000,
          0.0,
          0.0,
          0.0,
          1000.0,
          1000.0,
          1000.0,
          1.0, // 4.846
          Angle.ofBaseUnits(0.0, Degrees),
          40,
          80);

  public static final WristConstants HangWrist =
      new WristConstants(
          15,
          InvertedValue.CounterClockwise_Positive,
          90,
          1.369162,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          10000.0,
          30.0 * 3,
          10000000.0,
          1.0,
          Angle.ofBaseUnits(0.0, Degrees),
          340,
          340); // 28.64 rot
}
