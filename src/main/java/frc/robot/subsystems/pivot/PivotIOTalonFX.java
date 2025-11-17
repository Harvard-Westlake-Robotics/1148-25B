package frc.robot.subsystems.pivot;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
  private TalonFX masterMotor;
  private TalonFX followerMotor1;
  private TalonFX followerMotor2;
  private MotionMagicVoltage pivotController;

  private TalonFXConfiguration pivotConfig;

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorsConnectedDebouncer = new Debouncer(0.5);

  public PivotIOTalonFX() {
    masterMotor = new TalonFX(PivotConstants.motor1Id, "drive");
    followerMotor1 = new TalonFX(PivotConstants.motor2Id, "drive");
    followerMotor2 = new TalonFX(PivotConstants.motor3Id, "drive");

    masterMotor.setPosition(PivotConstants.angleOffset);
    followerMotor1.setPosition(PivotConstants.angleOffset);
    followerMotor2.setPosition(PivotConstants.angleOffset);

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
    this.masterMotor.getConfigurator().apply(this.pivotConfig);
    masterMotor.setControl(pivotController);

    this.followerMotor1.getConfigurator().apply(this.pivotConfig);
    followerMotor1.setControl(new Follower(PivotConstants.motor1Id, false));

    this.followerMotor2.getConfigurator().apply(this.pivotConfig);
    followerMotor2.setControl( new Follower(PivotConstants.motor1Id, false));

    pivotPosition = masterMotor.getPosition();
    motorVelocity = masterMotor.getVelocity();
    pivotAppliedVolts = masterMotor.getMotorVoltage();
    motorCurrent = masterMotor.getStatorCurrent();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    StatusSignal.refreshAll(pivotPosition, motorVelocity, pivotAppliedVolts, motorCurrent);

    inputs.pivotMotor1Connected = motorsConnectedDebouncer.calculate(masterMotor.isConnected());
    inputs.pivotMotor2Connected = motorsConnectedDebouncer.calculate(followerMotor1.isConnected());
    inputs.pivotMotor3Connected = motorsConnectedDebouncer.calculate(followerMotor2.isConnected());
    inputs.pivotPositionDeg = Units.rotationsToDegrees(pivotPosition.getValueAsDouble());
    inputs.pivotVelocityDPS = Units.rotationsToDegrees(motorVelocity.getValueAsDouble());
    inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.pivotCurrentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void runVoltage(double voltage) {
    masterMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void goToAngleClosedLoop(double angle) {
    masterMotor.setControl(pivotController.withPosition(angle));
  }

  @Override
  public void tareAngle(double angle) {
    masterMotor.setPosition(angle);
    followerMotor1.setPosition(angle);
    followerMotor2.setPosition(angle);
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
    tryUntilOk(5, () -> masterMotor.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> followerMotor1.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> followerMotor2.getConfigurator().apply(pivotConfig, 0.25));
    masterMotor.setControl(pivotController.withPosition(pivotAngle));
  }

  @Override
  public double getPivotTargetDegrees() {
    return Units.rotationsToDegrees(pivotController.Position);
  }
}
