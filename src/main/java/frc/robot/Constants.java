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

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
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
  public static final IntakeConstants AlgaeIntake = new IntakeConstants(
      15,
      InvertedValue.Clockwise_Positive,
      100,
      -100,
      0.369162,
      0.0,
      0.0,
      0.1761,
      0.12875,
      0.0,
      100.0,
      1000.0,
      10000.0,
      1.0 / 16.709);

  public static final IntakeConstants CoralIntake = new IntakeConstants(
      16,
      InvertedValue.Clockwise_Positive,
      100,
      -100,
      0.369162,
      0.0,
      0.0,
      0.1761,
      0.12875,
      0.0,
      100.0,
      1000.0,
      10000.0,
      1.0 / 16.709);

  public final class Elevator {
    public static final int elevator1ID = 18;
    public static final int elevator2ID = 19;
    public static final InvertedValue elevator2Inverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue elevator1Inverted = InvertedValue.CounterClockwise_Positive;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 0;
    public static double kV = 0;
    public static double kG = 0;
    public static double kA = 0;
    public static final double elevatorForwardSoftLimitRotations = 25;
    public static final double elevatorReverseSoftLimitRotations = 0;
    public static final double rotationsToMetersRatio = 1; // TODO: Define
    public static final double[] elevatorHeights = { 0, 1, 2, 3 };
  }

  public final class Hang {
    public static final int motorId = 0;
    public static final boolean motorInverted = false;
    public static final double hangVelocity = 0;
    public static final double unhungPosition = 0;
    public static final double hungPosition = 0;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kG = 0.0;
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

    public final double ANGLE_MAX_ACCELERATION;
    public final double ANGLE_MAX_VELOCITY;
    public final double ANGLe_MAX_JERK;
    public final double motorToWristRotations;

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
        double motorToWristRotations) {
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
    }
  }

  // Fix later with real values
  public static final WristConstants IntakeWrist = new WristConstants(0, InvertedValue.Clockwise_Positive, 100,
      0.369162, 0.0, 0.0, 0.1761, 0.12875, 0.0, 0.0, 100.0, 1000.0, 10000.0, 1.0 / 4.846);
}
