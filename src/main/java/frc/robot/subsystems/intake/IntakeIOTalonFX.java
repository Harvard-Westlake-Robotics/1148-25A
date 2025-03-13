package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
  private final IntakeConstants intakeConstants;
  // Motors and intake controllers
  private TalonFX intakeMotor;
  private MotionMagicVelocityTorqueCurrentFOC intakeController;
  private MotionMagicVoltage intakePositionController;
  private VoltageOut voltageRequest = new VoltageOut(0);

  private SimpleMotorFeedforward intakeFeedforward;

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  private final DigitalInput input1;
  private final DigitalInput input2;
  private final DigitalInput input3;

  // Connection debouncers
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  public IntakeIOTalonFX(IntakeConstants intakeConstants) {
    this.intakeConstants = intakeConstants;
    intakeMotor = new TalonFX(intakeConstants.motorId);
    intakeMotor.setPosition(0);
    intakeController =
        new MotionMagicVelocityTorqueCurrentFOC(
            AngularVelocity.ofBaseUnits(0.0, RotationsPerSecond));
    intakePositionController = new MotionMagicVoltage(0.0).withSlot(1).withEnableFOC(true);
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted = intakeConstants.motorInverted;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotionMagic.MotionMagicAcceleration = intakeConstants.ANGLE_MAX_ACCELERATION;
    intakeConfig.MotionMagic.MotionMagicCruiseVelocity = intakeConstants.ANGLE_MAX_VELOCITY;
    intakeConfig.MotionMagic.MotionMagicJerk = 0;
    intakeConfig.Slot0.kP = intakeConstants.kP;
    intakeConfig.Slot0.kI = intakeConstants.kI;
    intakeConfig.Slot0.kD = intakeConstants.kD;
    intakeConfig.Slot0.kS = intakeConstants.kS;
    intakeConfig.Slot0.kA = intakeConstants.kA;
    intakeConfig.Slot0.kV = intakeConstants.kV;
    intakeConfig.Slot1.kP = intakeConstants.positionkP;
    intakeConfig.Slot1.kD = intakeConstants.positionkP;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 90;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
    intakeMotor.getConfigurator().apply(intakeConfig);
    intakeMotor.setControl(intakeController);

    motorPosition = intakeMotor.getPosition();
    motorVelocity = intakeMotor.getVelocity();
    motorAppliedVolts = intakeMotor.getMotorVoltage();
    motorCurrent = intakeMotor.getStatorCurrent();

    this.input1 =
        intakeConstants.sensor1ID != -1 ? new DigitalInput(intakeConstants.sensor1ID) : null;
    this.input2 =
        intakeConstants.sensor2ID != -1 ? new DigitalInput(intakeConstants.sensor2ID) : null;
    this.input3 =
        intakeConstants.sensor3ID != -1 ? new DigitalInput(intakeConstants.sensor3ID) : null;

    intakeFeedforward =
        new SimpleMotorFeedforward(intakeConstants.kS, intakeConstants.kA, intakeConstants.kV);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var intakeStatus =
        StatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.intakeMotorConnected = motorConnectedDebounce.calculate(intakeMotor.isConnected());
    inputs.intakePositionMeters =
        motorPosition.getValueAsDouble() * Constants.IntakeWrist.motorToWristRotations;
    inputs.intakeVelocityMPS =
        motorVelocity.getValueAsDouble() * Constants.IntakeWrist.motorToWristRotations;
    inputs.intakeAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.intakeCurrentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void runCharacterization(double voltage) {
    intakeMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void runVelocity(LinearVelocity velocity) {
    intakeController.FeedForward =
        intakeFeedforward.calculate(
            velocity.in(MetersPerSecond) / intakeConstants.rotationsToMetersRatio);
    intakeController.Velocity =
        velocity.in(MetersPerSecond) / intakeConstants.rotationsToMetersRatio;
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
    voltageRequest =
        new VoltageOut(
            intakeFeedforward.calculate(
                velocity.in(MetersPerSecond) / intakeConstants.rotationsToMetersRatio));
    intakeMotor.setControl(voltageRequest);
  }

  @Override
  public void runOpenLoop(AngularVelocity velocity) {
    voltageRequest = new VoltageOut(intakeFeedforward.calculate(velocity.in(RotationsPerSecond)));
    intakeMotor.setControl(voltageRequest);
  }

  public void push(double rotations) {
    intakeMotor.setPosition(0);
    intakePositionController.Position = rotations;
    intakePositionController.Slot = 1;
    intakeMotor.setControl(intakePositionController);
  }

  public Boolean getSensor3() {
    if (input3 != null) {
      return input2.get();
    } else {
      return null;
    }
  }

  public Boolean getSensor2() {
    if (input2 != null) {
      return input2.get();
    } else {
      return null;
    }
  }

  public Boolean getSensor1() {
    if (input1 != null) {
      return input1.get();
    } else {
      return null;
    }
  }
}
