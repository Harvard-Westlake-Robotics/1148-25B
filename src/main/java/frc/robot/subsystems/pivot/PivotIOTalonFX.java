package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
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
  private TalonFX pivotMotor;
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

  public PivotIOTalonFX(int motorNum, String canbus) {
    pivotMotor = new TalonFX(PivotConstants.motorId + motorNum - 1, canbus);
    cancoder = new CANcoder(PivotConstants.pivotEncoderId, "drive");

    pivotMotor.setPosition(getAnglePivotRots());
    this.pivotController =
        new MotionMagicVoltage(0).withEnableFOC(true).withPosition(PivotConstants.angleOffset);
    pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.Inverted = PivotConstants.motorInverted;
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
    // pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    //     PivotConstants.pivotMaxAngle.in(Rotations);
    // pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    //     PivotConstants.pivotMinAngle.in(Rotations) *
    // PivotConstants.motorRotationsPerpivotRotationRatio;
    // System.out.println(
    //     PivotConstants.pivotMinAngle.in(Rotations) *
    // PivotConstants.motorRotationsPerpivotRotationRatio);
    this.pivotConfig.MotionMagic.MotionMagicAcceleration = PivotConstants.motionMagicAcceleration;
    this.pivotConfig.MotionMagic.MotionMagicCruiseVelocity =
        PivotConstants.motionMagicCruiseVelocity;
    this.pivotConfig.MotionMagic.MotionMagicJerk = PivotConstants.motionMagicJerk;
    this.pivotMotor.getConfigurator().apply(this.pivotConfig);
    pivotMotor.setControl(pivotController);

    encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = PivotConstants.pivotEncoderSensorDirection;
    encoderConfig.MagnetSensor.MagnetOffset = PivotConstants.pivotEncoderOffset;
    cancoder.getConfigurator().apply(encoderConfig);

    // TODO: Check that this works
    pivotPosition = cancoder.getPosition();
    pivotVelocity = cancoder.getVelocity();
    motorAppliedVolts = pivotMotor.getMotorVoltage();
    motorCurrent = pivotMotor.getStatorCurrent();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    StatusSignal.refreshAll(pivotPosition, pivotVelocity, motorAppliedVolts, motorCurrent);

    inputs.pivotEncoderConnected = encoderConnectedDebouncer.calculate(cancoder.isConnected());
    inputs.pivotMotorConnected = motorConnectedDebouncer.calculate(pivotMotor.isConnected());
    inputs.pivotPositionDeg =
        Units.rotationsToDegrees(
            cancoder.getPosition().getValueAsDouble()
                / PivotConstants.motorRotationsPerPivotRotationRatio);
    inputs.pivotVelocityDPS =
        Units.rotationsToDegrees(
            cancoder.getVelocity().getValueAsDouble()
                / PivotConstants.motorRotationsPerPivotRotationRatio);
    inputs.pivotAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.pivotCurrentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void runCharacterization(double voltage) {
    pivotMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void goToAngleClosedLoop(double angle) {
    pivotMotor.setControl(
        pivotController.withPosition(angle * PivotConstants.motorRotationsPerPivotRotationRatio));
  }

  @Override
  public void tareAngle(double angle) {
    pivotMotor.setPosition(angle * PivotConstants.motorRotationsPerPivotRotationRatio);
  }

  @Override
  public double getTargetDegrees() {
    return Units.rotationsToDegrees(
        pivotController.Position / PivotConstants.motorRotationsPerPivotRotationRatio);
  }

  public double getAnglePivotRots() {
    return cancoder.getPosition().getValueAsDouble()
        / PivotConstants.motorRotationsPerPivotRotationRatio;
  }
}
