package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.WristConstants;

public class WristIOTalonFX implements WristIO {
  // Motors and wrist controllers
  private TalonFX wristMotor;
  private MotionMagicVoltage wristController;

  private TalonFXConfiguration wristConfig;
  private ArmFeedforward feedforward;

  private final StatusSignal<Angle> wristPosition;
  private final StatusSignal<AngularVelocity> wristVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
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
    wristConfig.CurrentLimits.StatorCurrentLimit = WristConstants.statorLimit;
    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristConfig.CurrentLimits.SupplyCurrentLimit = WristConstants.supplyLimit;
    wristConfig.MotionMagic.MotionMagicAcceleration = WristConstants.motionMagicAcceleration;
    wristConfig.MotionMagic.MotionMagicCruiseVelocity = WristConstants.motionMagicCruiseVelocity;
    wristConfig.MotionMagic.MotionMagicJerk = WristConstants.motionMagicJerk;

    wristConfig.Feedback.SensorToMechanismRatio =
        WristConstants.motorRotationsPerWristRotationRatio;
    tryUntilOk(5, () -> wristMotor.getConfigurator().apply(wristConfig, 0.25));
    feedforward = new ArmFeedforward(WristConstants.kS, WristConstants.kG, WristConstants.kV);
    wristMotor.setPosition(WristConstants.angleOffset);
    wristMotor.setControl(wristController);

    wristPosition = wristMotor.getPosition();
    wristVelocity = wristMotor.getVelocity();
    motorAppliedVolts = wristMotor.getMotorVoltage();
    motorCurrent = wristMotor.getStatorCurrent();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    StatusSignal.refreshAll(wristPosition, wristVelocity, motorAppliedVolts, motorCurrent);

    inputs.wristMotorConnected = motorConnectedDebouncer.calculate(wristMotor.isConnected());
    inputs.wristPositionDeg = wristPosition.getValue().in(Degrees);
    inputs.wristVelocityDPS = wristVelocity.getValue().in(DegreesPerSecond);
    inputs.wristAppliedVolts = motorAppliedVolts.getValue().in(Volts);
    inputs.wristCurrentAmps = motorCurrent.getValue().in(Amps);
  }

  @Override
  public void runVoltage(double voltage) {
    wristMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void goToAngleClosedLoop(double wristAngleRots) {
    wristMotor.setControl(
        wristController
            .withPosition(wristAngleRots)
            .withFeedForward(
                feedforward.calculate(wristAngleRots, wristVelocity.getValueAsDouble())));
  }

  @Override
  public void tareAngle(double wristAngleRots) {
    wristMotor.setPosition(wristAngleRots);
  }

  @Override
  public double getTargetDegrees() {
    return Units.rotationsToDegrees(wristController.Position);
  }
}
