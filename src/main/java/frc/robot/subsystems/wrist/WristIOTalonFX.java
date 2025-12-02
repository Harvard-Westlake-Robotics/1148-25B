package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.pivot.Pivot;

public class WristIOTalonFX implements WristIO {
  // Motors and wrist controllers
  private TalonFX wristMotor;
  private MotionMagicVoltage wristController;

  private TalonFXConfiguration wristConfig;

  private final StatusSignal<Angle> wristPosition;
  private final StatusSignal<AngularVelocity> wristVelocity;
  private final StatusSignal<Voltage> motorAppliedVoltage;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  public WristIOTalonFX() {
    wristMotor = new TalonFX(WristConstants.motorId, WristConstants.motorCANBusName);
    this.wristController =
        new MotionMagicVoltage(0).withEnableFOC(true).withPosition(WristConstants.angleOffset);
    wristConfig = new TalonFXConfiguration();
    wristConfig.MotorOutput.Inverted = WristConstants.motorInverted;
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    wristConfig.Feedback.RotorToSensorRatio = 1.0;
    wristConfig.Feedback.SensorToMechanismRatio =
        WristConstants.motorRotationsPerWristRotationRatio;

    wristConfig.Slot0.kP = WristConstants.kP;
    wristConfig.Slot0.kI = WristConstants.kI;
    wristConfig.Slot0.kD = WristConstants.kD;
    wristConfig.Slot0.kS = WristConstants.kS;
    wristConfig.Slot0.kV = WristConstants.kV;
    wristConfig.Slot0.kA = WristConstants.kA;

    wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristConfig.CurrentLimits.StatorCurrentLimit = WristConstants.statorLimit.in(Amps);
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfig.CurrentLimits.SupplyCurrentLimit = WristConstants.supplyLimit.in(Amps);
    wristConfig.MotionMagic.MotionMagicAcceleration = WristConstants.motionMagicAcceleration.in(RotationsPerSecondPerSecond);
    wristConfig.MotionMagic.MotionMagicCruiseVelocity = WristConstants.motionMagicCruiseVelocity.in(RotationsPerSecond);
    wristConfig.MotionMagic.MotionMagicJerk = WristConstants.motionMagicJerk.in(RotationsPerSecondPerSecond.per(Second));

    wristConfig.Feedback.SensorToMechanismRatio =
        WristConstants.motorRotationsPerWristRotationRatio;
    tryUntilOk(5, () -> wristMotor.getConfigurator().apply(wristConfig, 0.25));
    wristMotor.setPosition(WristConstants.angleOffset);
    wristMotor.setControl(wristController);

    wristPosition = wristMotor.getPosition();
    wristVelocity = wristMotor.getVelocity();
    motorAppliedVoltage = wristMotor.getMotorVoltage();
    motorCurrent = wristMotor.getStatorCurrent();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    StatusSignal.refreshAll(wristPosition, wristVelocity, motorAppliedVoltage, motorCurrent);

    inputs.wristMotorConnected = motorConnectedDebouncer.calculate(wristMotor.isConnected());
    inputs.wristAngle = wristPosition.getValue();
    inputs.wristVelocity = wristVelocity.getValue();
    inputs.wristAppliedVoltage = motorAppliedVoltage.getValue();
    inputs.wristCurrent = motorCurrent.getValue();
  }

  @Override
  public void runVoltage(Voltage voltage) {
    wristMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void goToAngleClosedLoop(Angle wristAngle) {
    wristMotor.setControl(
        wristController
            .withPosition(wristAngle)
            .withFeedForward(
              Math.cos(Pivot.getInstance().getAngle().in(Radians) + wristPosition.getValue().in(Radians)) *  WristConstants.kG
            ));
  }

  @Override
  public void tareAngle(Angle wristAngle) {
    wristMotor.setPosition(wristAngle);
  }

  @Override
  public Angle getTarget() {
    return Rotations.of(wristController.Position);
  }
}
