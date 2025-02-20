package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

public class WristIOTalonFX implements WristIO {
  // Motors and wrist controllers
  private TalonFX intakeWristMotor;
  private PositionVoltage intakeWristController;

  private ArmFeedforward intakeWristFeedforward;

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  public WristIOTalonFX(WristConstants intakeWrist) {
    intakeWristMotor = new TalonFX(intakeWrist.motorId);
    intakeWristMotor.setPosition(intakeWrist.angleOffset);
    intakeWristController = new PositionVoltage(0);
    TalonFXConfiguration intakeWristConfig = new TalonFXConfiguration();
    intakeWristConfig.MotorOutput.Inverted = intakeWrist.motorInverted;
    intakeWristConfig.MotionMagic.MotionMagicAcceleration = intakeWrist.ANGLE_MAX_ACCELERATION;
    intakeWristConfig.MotionMagic.MotionMagicCruiseVelocity = intakeWrist.ANGLE_MAX_VELOCITY;
    intakeWristConfig.MotionMagic.MotionMagicJerk = intakeWrist.ANGLe_MAX_JERK;
    intakeWristConfig.Slot0.kP = intakeWrist.kP;
    intakeWristConfig.Slot0.kI = intakeWrist.kI;
    intakeWristConfig.Slot0.kD = intakeWrist.kD;
    intakeWristConfig.Slot0.kS = intakeWrist.kS;
    intakeWristConfig.Slot0.kG = intakeWrist.kG;
    intakeWristConfig.Slot0.kV = intakeWrist.kV;
    intakeWristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeWristConfig.CurrentLimits.StatorCurrentLimit = 240;
    intakeWristConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    intakeWristConfig.CurrentLimits.SupplyCurrentLimit = 40;
    intakeWristMotor.getConfigurator().apply(intakeWristConfig);
    intakeWristMotor.setControl(intakeWristController);

    motorPosition = intakeWristMotor.getPosition();
    motorVelocity = intakeWristMotor.getVelocity();
    motorAppliedVolts = intakeWristMotor.getMotorVoltage();
    motorCurrent = intakeWristMotor.getStatorCurrent();

    intakeWristFeedforward =
        new ArmFeedforward(intakeWrist.kS, intakeWrist.kV, intakeWrist.kG, intakeWrist.kA);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    var wristStatus =
        StatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.wristMotorConnected = motorConnectedDebounce.calculate(intakeWristMotor.isConnected());
    inputs.wristPositionMeters =
        motorPosition.getValueAsDouble() * Constants.IntakeWrist.motorToWristRotations;
    inputs.wristVelocityMPS =
        motorVelocity.getValueAsDouble() * Constants.IntakeWrist.motorToWristRotations;
    inputs.wristAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.wristCurrentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void runCharacterization(double voltage) {
    intakeWristMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void setAngle(double angle) {
    intakeWristController.Position = angle * Constants.IntakeWrist.motorToWristRotations;
    // intakeWristController.FeedForward =
    //     intakeWristFeedforward.calculate(
    //         angle * 2 * 3.14159265358924,
    //         motorVelocity.getValueAsDouble() / Constants.IntakeWrist.motorToWristRotations);
    intakeWristMotor.setControl(intakeWristController);
  }
}
