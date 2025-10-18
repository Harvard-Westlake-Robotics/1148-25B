package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
  // Motors and elevator controllers
  private final TalonFX elevator1;
  private final TalonFX elevator2;
  private final MotionMagicVoltage elevatorController;
  private final MotionMagicVoltage elevator2Controller;

  private final ElevatorFeedforward elevatorFeedforward;

  private final StatusSignal<Angle> motor1Position;
  private final StatusSignal<AngularVelocity> motor1Velocity;
  private final StatusSignal<Voltage> motor1AppliedVolts;
  private final StatusSignal<Current> motor1Current;

  private final StatusSignal<Angle> motor2Position;
  private final StatusSignal<AngularVelocity> motor2Velocity;
  private final StatusSignal<Voltage> motor2AppliedVolts;
  private final StatusSignal<Current> motor2Current;

  // Connection debouncers
  private final Debouncer motor1ConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer motor2ConnectedDebouncer = new Debouncer(0.5);

  public ElevatorIOTalonFX() {
    elevator1 = new TalonFX(ElevatorConstants.elevator1ID);
    elevator2 = new TalonFX(ElevatorConstants.elevator2ID);
    // TODO: Magic number?
    elevator1.setPosition(25);
    elevator2.setPosition(25);
    elevatorController = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
    elevator2Controller = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
    TalonFXConfiguration elevator1Config = new TalonFXConfiguration();
    TalonFXConfiguration elevator2Config = new TalonFXConfiguration();
    elevator1Config.MotorOutput.Inverted = ElevatorConstants.elevator1Inverted;
    elevator2Config.MotorOutput.Inverted = ElevatorConstants.elevator2Inverted;

    elevator1Config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.motionMagicAcceleration;
    elevator1Config.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.motionMagicCruiseVelocity;
    elevator1Config.MotionMagic.MotionMagicJerk = ElevatorConstants.motionMagicJerk;

    elevator2Config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.motionMagicAcceleration;
    elevator2Config.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.motionMagicCruiseVelocity;
    elevator2Config.MotionMagic.MotionMagicJerk = ElevatorConstants.motionMagicJerk;

    elevator1Config.Slot0.kP = ElevatorConstants.kP;
    elevator1Config.Slot0.kI = ElevatorConstants.kI;
    elevator1Config.Slot0.kD = ElevatorConstants.kD;

    elevator2Config.Slot0.kP = ElevatorConstants.kP;
    elevator2Config.Slot0.kI = ElevatorConstants.kI;
    elevator2Config.Slot0.kD = ElevatorConstants.kD;

    elevator1Config.CurrentLimits.StatorCurrentLimitEnable = true;
    elevator1Config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.statorLimit;
    elevator1Config.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevator1Config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.supplyLimit;
    elevator2Config.CurrentLimits.StatorCurrentLimitEnable = true;
    elevator2Config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.statorLimit;
    elevator2Config.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevator2Config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.supplyLimit;
    elevator1Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevator1Config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.elevatorForwardSoftLimitRotations;
    elevator1Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevator1Config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ElevatorConstants.elevatorReverseSoftLimitRotations;
    elevator2Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevator2Config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.elevatorForwardSoftLimitRotations;
    elevator2Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevator2Config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ElevatorConstants.elevatorReverseSoftLimitRotations;
    elevator1.getConfigurator().apply(elevator1Config);
    elevator2.getConfigurator().apply(elevator2Config);
    elevator1.setControl(elevatorController);
    elevator2.setControl(elevator2Controller);

    motor1Position = elevator1.getPosition();
    motor1Velocity = elevator1.getVelocity();
    motor1AppliedVolts = elevator1.getMotorVoltage();
    motor1Current = elevator1.getStatorCurrent();

    motor2Position = elevator2.getPosition();
    motor2Velocity = elevator2.getVelocity();
    motor2AppliedVolts = elevator2.getMotorVoltage();
    motor2Current = elevator2.getStatorCurrent();

    elevatorFeedforward =
        new ElevatorFeedforward(
            ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    StatusSignal.refreshAll(motor1Position, motor1Velocity, motor1AppliedVolts, motor1Current);
    StatusSignal.refreshAll(motor2Position, motor2Velocity, motor2AppliedVolts, motor2Current);

    inputs.elevator1Connected = motor1ConnectedDebouncer.calculate(elevator1.isConnected());
    inputs.elevator1PositionMeters =
        motor1Position.getValueAsDouble() / ElevatorConstants.rotationsToMetersRatio;
    inputs.elevator1VelocityMPS =
        motor1Velocity.getValueAsDouble() / ElevatorConstants.rotationsToMetersRatio;
    inputs.elevator1AppliedVolts = motor1AppliedVolts.getValueAsDouble();
    inputs.elevator1CurrentAmps = motor1Current.getValueAsDouble();

    inputs.elevator2Connected = motor2ConnectedDebouncer.calculate(elevator2.isConnected());
    inputs.elevator2PositionMeters =
        motor2Position.getValueAsDouble() / ElevatorConstants.rotationsToMetersRatio;
    inputs.elevator2VelocityMPS =
        motor2Velocity.getValueAsDouble() / ElevatorConstants.rotationsToMetersRatio;
    inputs.elevator2AppliedVolts = motor2AppliedVolts.getValueAsDouble();
    inputs.elevator2CurrentAmps = motor2Current.getValueAsDouble();
  }

  @Override
  public void runCharacterization(double voltage) {
    elevator1.setControl(new VoltageOut(voltage));
    elevator2.setControl(new VoltageOut(voltage));
  }

  @Override
  public void goToHeightClosedLoop(double height) {
    elevator1.setControl(
        elevatorController
            .withPosition(height * ElevatorConstants.rotationsToMetersRatio)
            .withFeedForward(elevatorFeedforward.calculate(0)));
    elevator2.setControl(
        elevator2Controller
            .withPosition(height * ElevatorConstants.rotationsToMetersRatio)
            .withFeedForward(elevatorFeedforward.calculate(0)));
  }

  @Override
  public void tareHeight(double height) {
    elevator1.setPosition(height * ElevatorConstants.rotationsToMetersRatio);
    elevator2.setPosition(height * ElevatorConstants.rotationsToMetersRatio);
  }

  public double getTarget() {
    return elevatorController.Position;
  }
}
