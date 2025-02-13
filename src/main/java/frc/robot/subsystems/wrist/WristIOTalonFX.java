package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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
  private final WristConstants intakeWrist;
  // Motors and wrist controllers
  private TalonFX intakeWristMotor;
  private MotionMagicTorqueCurrentFOC intakeWristController;
  private VoltageOut voltageRequest = new VoltageOut(0);

  private ArmFeedforward intakeWristFeedforward;

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  public WristIOTalonFX(WristConstants intakeWrist) {
    this.intakeWrist = intakeWrist;
    intakeWristMotor = new TalonFX(intakeWrist.motorId);
    intakeWristController = new MotionMagicTorqueCurrentFOC(0);
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
    intakeWristConfig.CurrentLimits.StatorCurrentLimit = 80;
    intakeWristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeWristConfig.CurrentLimits.SupplyCurrentLimit = 40;
    intakeWristMotor.getConfigurator().apply(intakeWristConfig);
    intakeWristMotor.setControl(intakeWristController);

    motorPosition = intakeWristMotor.getPosition();
    motorVelocity = intakeWristMotor.getVelocity();
    motorAppliedVolts = intakeWristMotor.getMotorVoltage();
    motorCurrent = intakeWristMotor.getStatorCurrent();

    intakeWristFeedforward = new ArmFeedforward(intakeWrist.kS, intakeWrist.kV, intakeWrist.kG, intakeWrist.kA);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    var wristStatus = StatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.wristMotorConnected = motorConnectedDebounce.calculate(intakeWristMotor.isConnected());
    inputs.wristPositionMeters = motorPosition.getValueAsDouble() * Constants.IntakeWrist.motorToWristRotations;
    inputs.wristVelocityMPS = motorVelocity.getValueAsDouble() * Constants.IntakeWrist.motorToWristRotations;
    inputs.wristAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.wristCurrentAmps = motorCurrent.getValueAsDouble();
  }

  public void runCharacterization(double voltage) {
    intakeWristMotor.setControl(new VoltageOut(voltage));
  }

  public void setAngle(Angle angle) {
    intakeWristController.Position = angle.in(Rotations) * Constants.IntakeWrist.motorToWristRotations;
    intakeWristController.FeedForward = intakeWristFeedforward.calculate(angle.in(Radians),
        motorVelocity.getValueAsDouble() / Constants.IntakeWrist.motorToWristRotations);

  }
}
