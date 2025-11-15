package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Velocity;
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
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> motorAppliedVoltage;
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

    pivotConfig.Feedback.RotorToSensorRatio = 1.0;
    pivotConfig.Feedback.SensorToMechanismRatio =
        PivotConstants.motorRotationsPerPivotRotationRatio;

    pivotConfig.Slot0.kP = PivotConstants.kP;
    pivotConfig.Slot0.kI = PivotConstants.kI;
    pivotConfig.Slot0.kD = PivotConstants.kD;
    pivotConfig.Slot0.kS = PivotConstants.kS.in(Volts);
    pivotConfig.Slot0.kV = PivotConstants.kV.in(Volts.per(RotationsPerSecond));
    pivotConfig.Slot0.kA = PivotConstants.kA.in(Volts.per(RotationsPerSecondPerSecond));
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = PivotConstants.statorLimit.in(Amps);
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = PivotConstants.supplyLimit.in(Amps);

    this.pivotConfig.MotionMagic.MotionMagicAcceleration = PivotConstants.motionMagicAcceleration.in(RotationsPerSecondPerSecond);
    this.pivotConfig.MotionMagic.MotionMagicCruiseVelocity =
        PivotConstants.motionMagicCruiseVelocity.in(RotationsPerSecond);
    this.pivotConfig.MotionMagic.MotionMagicJerk = PivotConstants.motionMagicJerk.in(RotationsPerSecondPerSecond.per(Second));
    
    this.pivotMotor1.getConfigurator().apply(this.pivotConfig);
    pivotMotor1.setControl(pivotController);

    this.pivotMotor2.getConfigurator().apply(this.pivotConfig);
    pivotMotor2.setControl(pivotController);

    this.pivotMotor3.getConfigurator().apply(this.pivotConfig);
    pivotMotor3.setControl(pivotController);

    pivotPosition = pivotMotor1.getPosition();
    pivotVelocity = pivotMotor1.getVelocity();
    motorAppliedVoltage = pivotMotor1.getMotorVoltage();
    motorCurrent = pivotMotor1.getStatorCurrent();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    StatusSignal.refreshAll(pivotPosition, pivotVelocity, motorAppliedVoltage, motorCurrent);

    inputs.pivotMotor1Connected = motorsConnectedDebouncer.calculate(pivotMotor1.isConnected());
    inputs.pivotMotor2Connected = motorsConnectedDebouncer.calculate(pivotMotor2.isConnected());
    inputs.pivotMotor3Connected = motorsConnectedDebouncer.calculate(pivotMotor3.isConnected());
    inputs.pivotAngle = pivotPosition.getValue();
    inputs.pivotVelocity = pivotVelocity.getValue();
    inputs.pivotAppliedVoltage = motorAppliedVoltage.getValue();
    inputs.pivotCurrent = motorCurrent.getValue();
  }

  @Override
  public void runVoltage(Voltage voltage) {
    pivotMotor1.setControl(new VoltageOut(voltage));
    pivotMotor2.setControl(new VoltageOut(voltage));
    pivotMotor3.setControl(new VoltageOut(voltage));
  }

  @Override
  public void goToAngleClosedLoop(Angle pivotAngle) {
    pivotMotor1.setControl(pivotController.withPosition(pivotAngle));
    pivotMotor2.setControl(pivotController.withPosition(pivotAngle));
    pivotMotor3.setControl(pivotController.withPosition(pivotAngle));
  }

  @Override
  public void tareAngle(Angle pivotAngle) {
    pivotMotor1.setPosition(pivotAngle);
    pivotMotor2.setPosition(pivotAngle);
    pivotMotor3.setPosition(pivotAngle);
  }

  @Override
  public void setTunableConstants(
      double kP,
      double kI,
      double kD,
      Voltage kS,
      Per<VoltageUnit, AngularVelocityUnit> kV,
      Voltage kG,
      Per<VoltageUnit, AngularAccelerationUnit> kA,
      AngularAcceleration motionMagicAcceleration,
      AngularVelocity motionMagicCruiseVelocity,
      Velocity<AngularAccelerationUnit> motionMagicJerk,
      Angle pivotAngle) {
    pivotConfig.Slot0.kP = kP;
    pivotConfig.Slot0.kI = kI;
    pivotConfig.Slot0.kD = kD;
    pivotConfig.Slot0.kS = kS.in(Volts);
    pivotConfig.Slot0.kV = kV.in(Volts.per(RotationsPerSecond));
    pivotConfig.Slot0.kG = kG.in(Volts);
    pivotConfig.Slot0.kA = kA.in(Volts.per(RotationsPerSecondPerSecond));
    pivotConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.in(RotationsPerSecondPerSecond);
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicCruiseVelocity.in(RotationsPerSecond);
    pivotConfig.MotionMagic.MotionMagicJerk = motionMagicJerk.in(RotationsPerSecondPerSecond.per(Second));
    tryUntilOk(5, () -> pivotMotor1.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> pivotMotor2.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> pivotMotor3.getConfigurator().apply(pivotConfig, 0.25));
    pivotMotor1.setControl(pivotController.withPosition(pivotAngle));
    pivotMotor2.setControl(pivotController.withPosition(pivotAngle));
    pivotMotor3.setControl(pivotController.withPosition(pivotAngle));
  }

  @Override
  public Angle getPivotTarget() {
    return Rotations.of(pivotController.Position);
  }
}
