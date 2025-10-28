package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  public WristIOTalonFX(int motorNum, String canbus) {
    wristMotor = new TalonFX(WristConstants.motorId + motorNum - 1, canbus);
    wristMotor.setPosition(WristConstants.angleOffset);
    this.wristController =
        new MotionMagicVoltage(0).withEnableFOC(true).withPosition(WristConstants.angleOffset);
    wristConfig = new TalonFXConfiguration();
    wristConfig.MotorOutput.Inverted = WristConstants.motorInverted;
    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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
    // wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    //     WristConstants.wristMaxAngle.in(Rotations);
    // wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    //     WristConstants.wristMinAngle.in(Rotations) *
    // WristConstants.motorRotationsPerWristRotationRatio;
    // System.out.println(
    //     WristConstants.wristMinAngle.in(Rotations) *
    // WristConstants.motorRotationsPerWristRotationRatio);
    this.wristConfig.MotionMagic.MotionMagicAcceleration = WristConstants.motionMagicAcceleration;
    this.wristConfig.MotionMagic.MotionMagicCruiseVelocity =
        WristConstants.motionMagicCruiseVelocity;
    this.wristConfig.MotionMagic.MotionMagicJerk = WristConstants.motionMagicJerk;
    this.wristMotor.getConfigurator().apply(this.wristConfig);
    wristMotor.setControl(wristController);

    motorPosition = wristMotor.getPosition();
    motorVelocity = wristMotor.getVelocity();
    motorAppliedVolts = wristMotor.getMotorVoltage();
    motorCurrent = wristMotor.getStatorCurrent();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    StatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.wristMotorConnected = motorConnectedDebouncer.calculate(wristMotor.isConnected());
    inputs.wristPositionDeg =
        Units.rotationsToDegrees(
            motorPosition.getValueAsDouble() / WristConstants.motorRotationsPerWristRotationRatio);
    inputs.wristVelocityDPS =
        Units.rotationsToDegrees(
            motorVelocity.getValueAsDouble() / WristConstants.motorRotationsPerWristRotationRatio);
    inputs.wristAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.wristCurrentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void runCharacterization(double voltage) {
    wristMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void goToAngleClosedLoop(double angle) {
    wristMotor.setControl(
        wristController.withPosition(angle * WristConstants.motorRotationsPerWristRotationRatio));
  }

  @Override
  public void tareAngle(double angle) {
    wristMotor.setPosition(angle * WristConstants.motorRotationsPerWristRotationRatio);
  }

  @Override
  public double getTargetDegrees() {
    return Units.rotationsToDegrees(
        wristController.Position / WristConstants.motorRotationsPerWristRotationRatio);
  }

  public double getAngle() {
    return wristMotor.getPosition().getValueAsDouble()
        / WristConstants.motorRotationsPerWristRotationRatio;
  }
}
