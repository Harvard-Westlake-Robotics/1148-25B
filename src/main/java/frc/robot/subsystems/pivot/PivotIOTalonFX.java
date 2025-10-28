package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

  private final CANcoder cancoder;
  private CANcoderConfiguration encoderConfig;

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5);

  public PivotIOTalonFX() {
    pivotMotor1 = new TalonFX(PivotConstants.motor1Id, PivotConstants.motorCANBusName);
    pivotMotor2 = new TalonFX(PivotConstants.motor2Id, PivotConstants.motorCANBusName);
    pivotMotor3 = new TalonFX(PivotConstants.motor3Id, PivotConstants.motorCANBusName);
    cancoder = new CANcoder(PivotConstants.pivotEncoderId, PivotConstants.pivotEncoderCANBusName);

    this.pivotController =
        new MotionMagicVoltage(0).withEnableFOC(true).withPosition(PivotConstants.angleOffset);
    pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.Inverted = PivotConstants.motor1Inverted;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfig.Slot0.kP = PivotConstants.kP;
    pivotConfig.Slot0.kI = PivotConstants.kI;
    pivotConfig.Slot0.kD = PivotConstants.kD;
    pivotConfig.Slot0.kS = PivotConstants.kS;
    pivotConfig.Slot0.kG = PivotConstants.kG;
    pivotConfig.Slot0.kV = PivotConstants.kV;
    pivotConfig.Slot0.kA = PivotConstants.kA;
    pivotConfig.Slot0.GravityType = PivotConstants.gravityType;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = PivotConstants.statorLimit;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = PivotConstants.supplyLimit;
    // pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    //     PivotConstants.pivotMaxAngle.in(Rotations);
    // pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    //     PivotConstants.pivotMinAngle.in(Rotations) *
    // PivotConstants.motorRotationsPerPivotRotationRatio;
    // System.out.println(
    //     PivotConstants.pivotMinAngle.in(Rotations) *
    // PivotConstants.motorRotationsPerPivotRotationRatio);
    pivotConfig.MotionMagic.MotionMagicAcceleration = PivotConstants.motionMagicAcceleration;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.motionMagicCruiseVelocity;
    pivotConfig.MotionMagic.MotionMagicJerk = PivotConstants.motionMagicJerk;

    pivotConfig.Feedback.FeedbackRemoteSensorID = PivotConstants.pivotEncoderId;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotConfig.Feedback.SensorToMechanismRatio =
        PivotConstants.pivotEncoderRotationsPerPivotRotationRatio;
    pivotConfig.Feedback.RotorToSensorRatio =
        PivotConstants.motorRotationsPerPivotRotationRatio
            / PivotConstants.pivotEncoderRotationsPerPivotRotationRatio;
    tryUntilOk(5, () -> pivotMotor1.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> pivotMotor2.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> pivotMotor3.getConfigurator().apply(pivotConfig, 0.25));
    pivotMotor1.setPosition(PivotConstants.angleOffset);
    pivotMotor2.setPosition(PivotConstants.angleOffset);
    pivotMotor3.setPosition(PivotConstants.angleOffset);
    pivotMotor1.setControl(pivotController);
    pivotMotor2.setControl(pivotController);
    pivotMotor3.setControl(pivotController);

    encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = PivotConstants.pivotEncoderSensorDirection;
    encoderConfig.MagnetSensor.MagnetOffset = PivotConstants.pivotEncoderOffset;
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(encoderConfig, 0.25));

    pivotPosition = pivotMotor1.getPosition();
    pivotVelocity = pivotMotor1.getVelocity();
    motorAppliedVolts = pivotMotor1.getMotorVoltage();
    motorCurrent = pivotMotor1.getStatorCurrent();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    StatusSignal.refreshAll(pivotPosition, pivotVelocity, motorAppliedVolts, motorCurrent);

    inputs.pivotMotorConnected = motorConnectedDebouncer.calculate(pivotMotor1.isConnected());
    inputs.pivotEncoderConnected = encoderConnectedDebouncer.calculate(cancoder.isConnected());
    inputs.pivotPositionDeg = pivotPosition.getValue().in(Degrees);
    inputs.pivotVelocityDPS = pivotVelocity.getValue().in(DegreesPerSecond);
    inputs.pivotAppliedVolts = motorAppliedVolts.getValue().in(Volts);
    inputs.pivotCurrentAmps = motorCurrent.getValue().in(Amps);
  }

  @Override
  public void runVoltage(double voltage) {
    pivotMotor1.setControl(new VoltageOut(voltage));
    pivotMotor2.setControl(new VoltageOut(voltage));
    pivotMotor3.setControl(new VoltageOut(voltage));
  }

  @Override
  public void goToAngleClosedLoop(double pivotAngleRots) {
    pivotMotor1.setControl(pivotController.withPosition(pivotAngleRots));
    pivotMotor2.setControl(pivotController.withPosition(pivotAngleRots));
    pivotMotor3.setControl(pivotController.withPosition(pivotAngleRots));
  }

  @Override
  public void tareAngle(double pivotAngleRots) {
    pivotMotor1.setPosition(pivotAngleRots);
    pivotMotor2.setPosition(pivotAngleRots);
    pivotMotor3.setPosition(pivotAngleRots);
  }

  @Override
  public double getPivotTargetDegrees() {
    return Units.rotationsToDegrees(pivotController.Position);
  }
}
