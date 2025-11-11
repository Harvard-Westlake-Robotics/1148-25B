package frc.robot.subsystems.pivot;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

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
import frc.robot.constants.PivotConstants;

public class PivotIOTalonFX implements PivotIO {
  // Motors and pivot controllers
  private TalonFX pivotMotor1;
  private TalonFX pivotMotor2;
  private TalonFX pivotMotor3;
  private MotionMagicVoltage pivotController;

  private TalonFXConfiguration pivotConfig;

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorsConnectedDebouncer = new Debouncer(0.5);

  public PivotIOTalonFX() {
    pivotMotor1 = new TalonFX(PivotConstants.motor1Id, "drive");
    pivotMotor2 = new TalonFX(PivotConstants.motor2Id, "drive");
    pivotMotor3 = new TalonFX(PivotConstants.motor3Id, "drive");

    pivotMotor1.setPosition(PivotConstants.angleOffset);
    pivotMotor2.setPosition(PivotConstants.angleOffset);
    pivotMotor3.setPosition(PivotConstants.angleOffset);

    this.pivotController =
        new MotionMagicVoltage(0).withEnableFOC(true).withPosition(PivotConstants.angleOffset);
    pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.Inverted = PivotConstants.motorsInverted;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfig.Slot0.kP = PivotConstants.kP;
    pivotConfig.Slot0.kI = PivotConstants.kI;
    pivotConfig.Slot0.kD = PivotConstants.kD;
    pivotConfig.Slot0.kS = PivotConstants.kS;
    pivotConfig.Slot0.kV = PivotConstants.kV;
    pivotConfig.Slot0.kA = PivotConstants.kA;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = PivotConstants.statorLimit;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = PivotConstants.supplyLimit;

    this.pivotConfig.MotionMagic.MotionMagicAcceleration = PivotConstants.motionMagicAcceleration;
    this.pivotConfig.MotionMagic.MotionMagicCruiseVelocity =
        PivotConstants.motionMagicCruiseVelocity;
    this.pivotConfig.MotionMagic.MotionMagicJerk = PivotConstants.motionMagicJerk;
    this.pivotMotor1.getConfigurator().apply(this.pivotConfig);
    pivotMotor1.setControl(pivotController);

    this.pivotMotor2.getConfigurator().apply(this.pivotConfig);
    pivotMotor2.setControl(pivotController);

    this.pivotMotor3.getConfigurator().apply(this.pivotConfig);
    pivotMotor3.setControl(pivotController);

    pivotPosition = pivotMotor1.getPosition();
    motorVelocity = pivotMotor1.getVelocity();
    pivotAppliedVolts = pivotMotor1.getMotorVoltage();
    motorCurrent = pivotMotor1.getStatorCurrent();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    StatusSignal.refreshAll(pivotPosition, motorVelocity, pivotAppliedVolts, motorCurrent);

    inputs.pivotMotor1Connected = motorsConnectedDebouncer.calculate(pivotMotor1.isConnected());
    inputs.pivotMotor2Connected = motorsConnectedDebouncer.calculate(pivotMotor2.isConnected());
    inputs.pivotMotor3Connected = motorsConnectedDebouncer.calculate(pivotMotor3.isConnected());
    inputs.pivotPositionDeg =
        Units.rotationsToDegrees(
            pivotPosition.getValueAsDouble() / PivotConstants.motorRotationsPerPivotRotationRatio);
    inputs.pivotVelocityDPS =
        Units.rotationsToDegrees(
            motorVelocity.getValueAsDouble() / PivotConstants.motorRotationsPerPivotRotationRatio);
    inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.pivotCurrentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void runVoltage(double voltage) {
    pivotMotor1.setControl(new VoltageOut(voltage));
    pivotMotor2.setControl(new VoltageOut(voltage));
    pivotMotor3.setControl(new VoltageOut(voltage));
  }

  @Override
  public void goToAngleClosedLoop(double angle) {
    pivotMotor1.setControl(
        pivotController.withPosition(angle * PivotConstants.motorRotationsPerPivotRotationRatio));
    pivotMotor2.setControl(
        pivotController.withPosition(angle * PivotConstants.motorRotationsPerPivotRotationRatio));
    pivotMotor3.setControl(
        pivotController.withPosition(angle * PivotConstants.motorRotationsPerPivotRotationRatio));
  }

  @Override
  public void tareAngle(double angle) {
    pivotMotor1.setPosition(angle * PivotConstants.motorRotationsPerPivotRotationRatio);
    pivotMotor2.setPosition(angle * PivotConstants.motorRotationsPerPivotRotationRatio);
    pivotMotor3.setPosition(angle * PivotConstants.motorRotationsPerPivotRotationRatio);
  }

  @Override
  public void setTunableConstants(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kG,
      double kA,
      double motionMagicAcceleration,
      double motionMagicCruiseVelocity,
      double motionMagicJerk,
      double pivotAngle) {
    pivotConfig.Slot0.kP = kP;
    pivotConfig.Slot0.kI = kI;
    pivotConfig.Slot0.kD = kD;
    pivotConfig.Slot0.kS = kS;
    pivotConfig.Slot0.kV = kV;
    pivotConfig.Slot0.kG = kG;
    pivotConfig.Slot0.kA = kA;
    pivotConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicCruiseVelocity;
    pivotConfig.MotionMagic.MotionMagicJerk = motionMagicJerk;
    tryUntilOk(5, () -> pivotMotor1.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> pivotMotor2.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> pivotMotor3.getConfigurator().apply(pivotConfig, 0.25));
    pivotMotor1.setControl(
        pivotController.withPosition(
            pivotAngle * PivotConstants.motorRotationsPerPivotRotationRatio));
    pivotMotor2.setControl(
        pivotController.withPosition(
            pivotAngle * PivotConstants.motorRotationsPerPivotRotationRatio));
    pivotMotor3.setControl(
        pivotController.withPosition(
            pivotAngle * PivotConstants.motorRotationsPerPivotRotationRatio));
  }

  @Override
  public double getPivotTargetDegrees() {
    return Units.rotationsToDegrees(
        pivotController.Position / PivotConstants.motorRotationsPerPivotRotationRatio);
  }
}
