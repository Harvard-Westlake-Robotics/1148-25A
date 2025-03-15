package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
  private MotionMagicVoltage intakeWristController;

  private WristConstants intakeWristOG;

  private TalonFXConfiguration intakeWristConfig;

  private ArmFeedforward intakeWristFeedforward;

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  public WristIOTalonFX(WristConstants intakeWrist) {
    this.intakeWristOG = intakeWrist;
    intakeWristMotor = new TalonFX(intakeWristOG.motorId);
    intakeWristMotor.setPosition(intakeWristOG.angleOffset);
    this.intakeWristController = new MotionMagicVoltage(0).withEnableFOC(true);
    intakeWristConfig = new TalonFXConfiguration();
    intakeWristConfig.MotorOutput.Inverted = intakeWristOG.motorInverted;
    intakeWristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeWristConfig.Slot0.kP = intakeWristOG.kP;
    intakeWristConfig.Slot0.kI = intakeWristOG.kI;
    intakeWristConfig.Slot0.kD = intakeWristOG.kD;
    intakeWristConfig.Slot0.kS = intakeWristOG.kS;
    intakeWristConfig.Slot0.kG = intakeWristOG.kG;
    intakeWristConfig.Slot0.kV = intakeWristOG.kV;
    intakeWristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeWristConfig.CurrentLimits.StatorCurrentLimit = intakeWristOG.statorLimit;
    intakeWristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeWristConfig.CurrentLimits.SupplyCurrentLimit = intakeWristOG.supplyLimit;
    this.intakeWristConfig.MotionMagic.MotionMagicAcceleration =
        this.intakeWristOG.ANGLE_MAX_ACCELERATION;
    this.intakeWristConfig.MotionMagic.MotionMagicCruiseVelocity =
        this.intakeWristOG.ANGLE_MAX_VELOCITY;
    this.intakeWristConfig.MotionMagic.MotionMagicJerk = this.intakeWristOG.ANGLe_MAX_JERK;
    this.intakeWristMotor.getConfigurator().apply(this.intakeWristConfig);
    this.intakeWristMotor
        .getConfigurator()
        .apply(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(intakeWristOG.ANGLE_MAX_VELOCITY)
                .withMotionMagicAcceleration(intakeWristOG.ANGLE_MAX_ACCELERATION));
    intakeWristMotor.setControl(intakeWristController);

    motorPosition = intakeWristMotor.getPosition();
    motorVelocity = intakeWristMotor.getVelocity();
    motorAppliedVolts = intakeWristMotor.getMotorVoltage();
    motorCurrent = intakeWristMotor.getStatorCurrent();

    intakeWristFeedforward =
        new ArmFeedforward(intakeWristOG.kS, intakeWristOG.kV, intakeWristOG.kG, intakeWristOG.kA);
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
    // intakeWristFeedforward.calculate(
    // angle * 2 * 3.14159265358924,
    // motorVelocity.getValueAsDouble() /
    // Constants.IntakeWrist.motorToWristRotations);
    intakeWristMotor.setControl(intakeWristController);
  }

  @Override
  public void zeroPosition(double rotations) {
    intakeWristMotor.setPosition(rotations);
  }
}
