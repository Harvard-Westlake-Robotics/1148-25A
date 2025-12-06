package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {
  private TalonFX motorMaster;
  private TalonFX motorFollower;

  private DigitalInput dio;

  private MotionMagicVoltage elevatorController;

  private StatusSignal<Angle> motorPosition;
  private StatusSignal<AngularVelocity> motorVelocity;
  private StatusSignal<Voltage> motorAppliedVolts;
  private StatusSignal<Current> motorCurrentAmps;

  private Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  public ElevatorIOTalonFX() {
    motorMaster = new TalonFX(Constants.Elevator.elevator1ID, "rio");
    motorFollower = new TalonFX(Constants.Elevator.elevator2ID, "rio");
    dio = new DigitalInput(5);

    elevatorController = new MotionMagicVoltage(0).withEnableFOC(true);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = Constants.Elevator.elevator1Inverted;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 50;

    config.Feedback.SensorToMechanismRatio = Constants.Elevator.rotationsToMetersRatio;

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kP = Constants.Elevator.kP;
    config.Slot0.kI = Constants.Elevator.kI;
    config.Slot0.kD = Constants.Elevator.kD;
    config.Slot0.kS = Constants.Elevator.kS;
    config.Slot0.kV = Constants.Elevator.kV;
    config.Slot0.kG = Constants.Elevator.kG;
    config.Slot0.kA = Constants.Elevator.kA;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.Elevator.elevatorForwardSoftLimitRotations;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.Elevator.elevatorReverseSoftLimitRotations;

    config.MotionMagic.MotionMagicAcceleration = 390;
    config.MotionMagic.MotionMagicCruiseVelocity = 250;
    config.MotionMagic.MotionMagicJerk = 990;

    motorMaster.getConfigurator().apply(config);
    motorFollower.getConfigurator().apply(config);

    motorMaster.setPosition(
        Constants.Elevator.elevatorGroundOffsetMeters * Constants.Elevator.rotationsToMetersRatio);
    motorFollower.setPosition(
        Constants.Elevator.elevatorGroundOffsetMeters * Constants.Elevator.rotationsToMetersRatio);

    motorMaster.setControl(elevatorController);
    motorFollower.setControl(
        new Follower(Constants.Elevator.elevator1ID, Constants.Elevator.elevatorMotorsOpposite));

    motorPosition = motorMaster.getPosition();
    motorVelocity = motorMaster.getVelocity();
    motorAppliedVolts = motorMaster.getMotorVoltage();
    motorCurrentAmps = motorMaster.getStatorCurrent();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrentAmps);

    inputs.motor1Connected = motorConnectedDebouncer.calculate(motorMaster.isConnected());
    inputs.motor2Connected = motorConnectedDebouncer.calculate(motorFollower.isConnected());

    inputs.elevatorPositionMeters =
        motorPosition.getValue().in(Rotations) / Constants.Elevator.rotationsToMetersRatio;
    inputs.elevatorVelocityMPS =
        motorVelocity.getValue().in(RotationsPerSecond) / Constants.Elevator.rotationsToMetersRatio;
    inputs.elevatorAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.elevatorCurrentAmps = motorCurrentAmps.getValueAsDouble();

    inputs.inputValue = dio.get();
  }

  @Override
  public void goToHeight(double heightMeters) {
    motorMaster.setControl(
        elevatorController.withPosition(heightMeters * Constants.Elevator.rotationsToMetersRatio));
  }

  @Override
  public double getTarget() {
    return elevatorController.Position;
  }

  @Override
  public void runVoltage(double volts) {
    motorMaster.setControl(new VoltageOut(volts));
  }
}
