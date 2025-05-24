package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.generated.TunerConstants;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ModuleIOSim implements ModuleIO {
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT = 0.91035;
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;
  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private final PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private final PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);

  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private double driveVelocitySetpointRadPerSec = 0.0;

  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    this.driveMotor = moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(60));
    this.turnMotor = moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(20));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    double currentSpeed = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + driveController.calculate(currentSpeed, driveVelocitySetpointRadPerSec);
    } else {
      driveController.reset();
    }

    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians());
    } else {
      turnController.reset();
    }

    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = currentSpeed;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(moduleSimulation.getSteerAbsoluteFacing().getRadians());
    inputs.turnPosition = new Rotation2d(moduleSimulation.getSteerAbsoluteFacing().getRadians());
    inputs.turnVelocityRadPerSec = moduleSimulation.getSteerRelativeEncoderVelocity().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

    inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
    inputs.odometryDrivePositionsRad = new double[] { inputs.drivePositionRad };
    inputs.odometryTurnPositions = new Rotation2d[] { inputs.turnPosition };

    setDriveOutputVoltage(Voltage.ofBaseUnits(driveAppliedVolts, Volts));
    setSteerOutputVoltage(Voltage.ofBaseUnits(turnAppliedVolts, Volts));
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }

  public void setDriveOutputVoltage(Voltage voltage) {
    this.driveMotor.requestVoltage(voltage);
  }

  public void setSteerOutputVoltage(Voltage voltage) {
    this.turnMotor.requestVoltage(voltage);
  }

  public Rotation2d getSteerFacing() {
    return this.moduleSimulation.getSteerAbsoluteFacing();
  }

  public Angle getSteerRelativePosition() {
    return moduleSimulation
        .getSteerRelativeEncoderPosition()
        .divide(TunerConstants.kSteerGearRatio);
  }

  public Angle getDriveWheelrPositiond() {
    return moduleSimulation.getDriveWheelFinalPosition();
  }
}
