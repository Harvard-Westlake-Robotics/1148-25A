package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
  private final IntakeConstants intakeConstants;
  // Motors and intake controllers
  private TalonFX intakeMotor;
  private MotionMagicVelocityTorqueCurrentFOC intakeController;
  private VoltageOut voltageRequest = new VoltageOut(0);

  private SimpleMotorFeedforward intakeFeedforward;

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  public IntakeIOTalonFX(IntakeConstants intakeConstants) {
    this.intakeConstants = intakeConstants;
    intakeMotor = new TalonFX(intakeConstants.motorId);
    intakeController = new MotionMagicVelocityTorqueCurrentFOC(
        AngularVelocity.ofBaseUnits(0.0, RotationsPerSecond));
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted = intakeConstants.motorInverted;
    intakeConfig.MotionMagic.MotionMagicAcceleration = intakeConstants.ANGLE_MAX_ACCELERATION;
    intakeConfig.MotionMagic.MotionMagicCruiseVelocity = intakeConstants.ANGLE_MAX_VELOCITY;
    intakeConfig.MotionMagic.MotionMagicJerk = intakeConstants.ANGLe_MAX_JERK;
    intakeConfig.Slot0.kP = intakeConstants.kP;
    intakeConfig.Slot0.kI = intakeConstants.kI;
    intakeConfig.Slot0.kD = intakeConstants.kD;
    intakeConfig.Slot0.kS = intakeConstants.kS;
    intakeConfig.Slot0.kA = intakeConstants.kA;
    intakeConfig.Slot0.kV = intakeConstants.kV;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 80;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
    intakeMotor.getConfigurator().apply(intakeConfig);
    intakeMotor.setControl(intakeController);

    motorPosition = intakeMotor.getPosition();
    motorVelocity = intakeMotor.getVelocity();
    motorAppliedVolts = intakeMotor.getMotorVoltage();
    motorCurrent = intakeMotor.getStatorCurrent();

    intakeFeedforward = new SimpleMotorFeedforward(
        intakeConstants.kS, intakeConstants.kA, intakeConstants.kV);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var intakeStatus = StatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.intakeMotorConnected = motorConnectedDebounce.calculate(intakeMotor.isConnected());
    inputs.intakePositionMeters = motorPosition.getValueAsDouble() * Constants.IntakeWrist.motorToWristRotations;
    inputs.intakeVelocityMPS = motorVelocity.getValueAsDouble() * Constants.IntakeWrist.motorToWristRotations;
    inputs.intakeAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.intakeCurrentAmps = motorCurrent.getValueAsDouble();
  }

  public void runCharacterization(double voltage) {
    intakeMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void runVelocity(LinearVelocity velocity) {
    intakeController.FeedForward = intakeFeedforward.calculate(
        velocity.in(MetersPerSecond) / intakeConstants.rotationsToMetersRatio);
    intakeController.Velocity = velocity.in(MetersPerSecond) / intakeConstants.rotationsToMetersRatio;
    intakeMotor.setControl(intakeController);
  }

  @Override
  public void runVelocity(AngularVelocity velocity) {
    intakeController.FeedForward = intakeFeedforward.calculate(velocity.in(RotationsPerSecond));
    intakeController.Velocity = velocity.in(RotationsPerSecond);
    intakeMotor.setControl(intakeController);
  }

  @Override
  public void runOpenLoop(LinearVelocity velocity) {
    voltageRequest = new VoltageOut(
        intakeFeedforward.calculate(
            velocity.in(MetersPerSecond) / intakeConstants.rotationsToMetersRatio));
    intakeMotor.setControl(voltageRequest);
  }

  @Override
  public void runOpenLoop(AngularVelocity velocity) {
    voltageRequest = new VoltageOut(intakeFeedforward.calculate(velocity.in(RotationsPerSecond)));
    intakeMotor.setControl(voltageRequest);
  }
}
