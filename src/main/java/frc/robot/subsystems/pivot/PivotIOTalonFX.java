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
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorsConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5);

  public PivotIOTalonFX() {
    pivotMotor1 = new TalonFX(PivotConstants.motor1Id, "drive");
    pivotMotor2 = new TalonFX(PivotConstants.motor2Id, "drive");
    pivotMotor3 = new TalonFX(PivotConstants.motor3Id, "drive");

    cancoder = new CANcoder(PivotConstants.pivotEncoderId, PivotConstants.pivotEncoderCANBusName);

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

    pivotConfig.Feedback.FeedbackRemoteSensorID = PivotConstants.pivotEncoderId;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotConfig.Feedback.SensorToMechanismRatio =
        PivotConstants.pivotEncoderRotationsPerPivotRotationRatio;
    pivotConfig.Feedback.RotorToSensorRatio =
        PivotConstants.motorRotationsPerPivotRotationRatio
            / PivotConstants.pivotEncoderRotationsPerPivotRotationRatio;

    this.pivotMotor1.getConfigurator().apply(this.pivotConfig);
    pivotMotor1.setControl(pivotController);

    this.pivotMotor2.getConfigurator().apply(this.pivotConfig);
    pivotMotor2.setControl(pivotController);

    this.pivotMotor3.getConfigurator().apply(this.pivotConfig);
    pivotMotor3.setControl(pivotController);

    encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = PivotConstants.pivotEncoderSensorDirection;
    encoderConfig.MagnetSensor.MagnetOffset = PivotConstants.pivotEncoderOffset;
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(encoderConfig, 0.25));

    pivotPosition = pivotMotor1.getPosition();
    pivotVelocity = pivotMotor1.getVelocity();
    pivotAppliedVolts = pivotMotor1.getMotorVoltage();
    motorCurrent = pivotMotor1.getStatorCurrent();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    StatusSignal.refreshAll(pivotPosition, pivotVelocity, pivotAppliedVolts, motorCurrent);

    inputs.pivotMotor1Connected = motorsConnectedDebouncer.calculate(pivotMotor1.isConnected());
    inputs.pivotMotor2Connected = motorsConnectedDebouncer.calculate(pivotMotor2.isConnected());
    inputs.pivotMotor3Connected = motorsConnectedDebouncer.calculate(pivotMotor3.isConnected());
    inputs.pivotEncoderConnected = encoderConnectedDebouncer.calculate(cancoder.isConnected());

    inputs.pivotPositionDeg = pivotPosition.getValue().in(Degrees);
    inputs.pivotVelocityDPS = pivotVelocity.getValue().in(DegreesPerSecond);
    inputs.pivotAppliedVolts = pivotAppliedVolts.getValue().in(Volts);
    inputs.pivotCurrentAmps = motorCurrent.getValue().in(Amps);
  }

  @Override
  public void runVoltage(double voltage) {
    pivotMotor1.setControl(new VoltageOut(voltage));
    pivotMotor2.setControl(new VoltageOut(voltage));
    pivotMotor3.setControl(new VoltageOut(voltage));
  }

  @Override
  public void goToAngleClosedLoop(double angle) {
    pivotMotor1.setControl(pivotController.withPosition(angle));
    pivotMotor2.setControl(pivotController.withPosition(angle));
    pivotMotor3.setControl(pivotController.withPosition(angle));
  }

  @Override
  public void tareAngle(double angle) {
    pivotMotor1.setPosition(angle);
    pivotMotor2.setPosition(angle);
    pivotMotor3.setPosition(angle);
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
    pivotMotor1.setControl(pivotController.withPosition(pivotAngle));
    pivotMotor2.setControl(pivotController.withPosition(pivotAngle));
    pivotMotor3.setControl(pivotController.withPosition(pivotAngle));
  }

  @Override
  public double getPivotTargetDegrees() {
    return Units.rotationsToDegrees(pivotController.Position);
  }
}
