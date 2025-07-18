package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
  private final IntakeConstants intakeConstants;
  // Motors and intake controllers
  private TalonFX intakeMotor;
  private MotionMagicVelocityTorqueCurrentFOC intakeController;
  private MotionMagicVoltage intakePositionController;
  private VoltageOut voltageRequest = new VoltageOut(0);

  private SimpleMotorFeedforward intakeFeedforward;

  private TalonFXConfiguration intakeConfig;

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  private final CANrange input1;
  private final CANrange input2;
  private final CANrange input3;
  private final CANrange input4;

  // Connection debouncers
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  public IntakeIOTalonFX(IntakeConstants intakeConstants, int motorNum) {
    this.intakeConstants = intakeConstants;
    intakeMotor = new TalonFX(intakeConstants.motorId + motorNum - 1);
    intakeMotor.setPosition(0);
    intakeController =
        new MotionMagicVelocityTorqueCurrentFOC(
            AngularVelocity.ofBaseUnits(0.0, RotationsPerSecond));
    intakePositionController = new MotionMagicVoltage(0.0).withSlot(1).withEnableFOC(true);
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    intakeConfig.MotorOutput.Inverted = intakeConstants.motorInverted;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotionMagic.MotionMagicAcceleration = intakeConstants.ANGLE_MAX_ACCELERATION;
    intakeConfig.MotionMagic.MotionMagicCruiseVelocity = intakeConstants.ANGLE_MAX_VELOCITY;
    intakeConfig.MotionMagic.MotionMagicJerk = 0;

    intakeConfig.Slot0.kP = intakeConstants.kP;
    intakeConfig.Slot0.kI = intakeConstants.kI;
    intakeConfig.Slot0.kD = intakeConstants.kD;
    intakeConfig.Slot0.kS = intakeConstants.kS;
    intakeConfig.Slot0.kA = intakeConstants.kA;
    intakeConfig.Slot0.kV = intakeConstants.kV;
    intakeConfig.Slot1.kP = intakeConstants.positionkP;
    intakeConfig.Slot1.kD = intakeConstants.positionkP;

    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 120;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 50;
    intakeMotor.getConfigurator().apply(intakeConfig);
    this.intakeConfig = intakeConfig;
    intakeMotor.setControl(intakeController);

    motorPosition = intakeMotor.getPosition();
    motorVelocity = intakeMotor.getVelocity();
    motorAppliedVolts = intakeMotor.getMotorVoltage();
    motorCurrent = intakeMotor.getStatorCurrent();

    if (motorNum == 1) {
      this.input1 =
          intakeConstants.sensor1ID != -1 ? new CANrange(intakeConstants.sensor1ID, "drive") : null;
      this.input2 =
          intakeConstants.sensor2ID != -1 ? new CANrange(intakeConstants.sensor2ID, "drive") : null;
      this.input3 =
          intakeConstants.sensor3ID != -1 ? new CANrange(intakeConstants.sensor3ID, "drive") : null;
      this.input4 =
          intakeConstants.sensor4ID != -1 ? new CANrange(intakeConstants.sensor4ID, "drive") : null;
    } else {
      this.input1 = null;
      this.input2 = null;
      this.input3 = null;
      this.input4 = null;
    }
    intakeFeedforward =
        new SimpleMotorFeedforward(intakeConstants.kS, intakeConstants.kA, intakeConstants.kV);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var intakeStatus =
        StatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.intakeMotorConnected = motorConnectedDebounce.calculate(intakeMotor.isConnected());
    inputs.intakeMotorPositionMeters =
        motorPosition.getValueAsDouble() * Constants.IntakeWrist.motorToWristRotations;
    inputs.intakeMotorVelocityMPS =
        motorVelocity.getValueAsDouble() * Constants.IntakeWrist.motorToWristRotations;
    inputs.intakeMotorAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.intakeMotorCurrentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void runCharacterization(double voltage) {
    intakeMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void runVelocity(LinearVelocity velocity) {
    intakeConfig.MotionMagic.MotionMagicCruiseVelocity = 9999;
    intakeConfig.MotionMagic.MotionMagicAcceleration = 9999;
    intakeMotor.getConfigurator().apply(intakeConfig);
    intakeController.FeedForward =
        intakeFeedforward.calculate(
            velocity.in(MetersPerSecond) / intakeConstants.rotationsToMetersRatio);
    intakeController.Velocity =
        velocity.in(MetersPerSecond) / intakeConstants.rotationsToMetersRatio;
    intakeMotor.setControl(intakeController);
  }

  @Override
  public void runVelocity(AngularVelocity velocity) {
    intakeController.FeedForward = intakeFeedforward.calculate(velocity.in(RotationsPerSecond));
    intakeController.Velocity = velocity.in(RotationsPerSecond);
    intakeMotor.setControl(intakeController);
  }

  public Boolean getSensor1() {
    if (input1 == null) return null;
    var distanceSignal = input1.getDistance();
    distanceSignal.refresh();
    double distance = distanceSignal.getValueAsDouble();
    if (distance != 0 && distance <= 0.02) {
      return true;
    } else {
      return false;
    }
  }

  public Boolean getSensor2() {
    if (input2 == null) return null;
    var distanceSignal = input2.getDistance();
    distanceSignal.refresh();
    double distance = distanceSignal.getValueAsDouble();
    if (distance != 0 && distance <= 0.02) {
      return true;
    } else {
      return false;
    }
  }

  public Boolean getSensor3() {
    if (input3 == null) return null;
    var distanceSignal = input3.getDistance();
    distanceSignal.refresh();
    double distance = distanceSignal.getValueAsDouble();
    if (distance != 0 && distance <= 0.02) {
      return true;
    } else {
      return false;
    }
  }

  public Boolean getSensor4() {
    if (input4 == null) return null;
    var distanceSignal = input4.getDistance();
    distanceSignal.refresh();
    double distance = distanceSignal.getValueAsDouble();
    if (distance != 0 && distance <= 0.02) {
      return true;
    } else {
      return false;
    }
  }
}
