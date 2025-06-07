package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {
  // Motors and elevator controllers
  private TalonFX elevator1;
  private TalonFX elevator2;
  private MotionMagicVoltage elevatorController;
  private MotionMagicVoltage elevator2Controller;
  private Follower follower;
  private VoltageOut voltageRequest = new VoltageOut(0);

  private ElevatorFeedforward elevatorFeedforward;

  private final StatusSignal<Angle> motor1Position;
  private final StatusSignal<AngularVelocity> motor1Velocity;
  private final StatusSignal<Voltage> motor1AppliedVolts;
  private final StatusSignal<Current> motor1Current;

  private final StatusSignal<Angle> motor2Position;
  private final StatusSignal<AngularVelocity> motor2Velocity;
  private final StatusSignal<Voltage> motor2AppliedVolts;
  private final StatusSignal<Current> motor2Current;

  // Connection debouncers
  private final Debouncer motor1ConnectedDebounce = new Debouncer(0.5);
  private final Debouncer motor2ConnectedDebounce = new Debouncer(0.5);

  public ElevatorIOTalonFX() {
    elevator1 = new TalonFX(Constants.Elevator.elevator1ID, "drive");
    elevator2 = new TalonFX(Constants.Elevator.elevator2ID, "drive");
    elevator1.setPosition(25);
    elevator2.setPosition(25);
    elevatorController = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
    elevator2Controller = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
    TalonFXConfiguration elevator1Config = new TalonFXConfiguration();
    TalonFXConfiguration elevator2Config = new TalonFXConfiguration();
    elevator1Config.MotorOutput.Inverted = Constants.Elevator.elevator1Inverted;
    elevator2Config.MotorOutput.Inverted = Constants.Elevator.elevator2Inverted;

    elevator1Config.MotionMagic.MotionMagicAcceleration = 390;
    elevator2Config.MotionMagic.MotionMagicAcceleration = 390;
    elevator1Config.MotionMagic.MotionMagicCruiseVelocity = 250;
    elevator2Config.MotionMagic.MotionMagicCruiseVelocity = 250;
    elevator1Config.MotionMagic.MotionMagicJerk = 990;
    elevator2Config.MotionMagic.MotionMagicJerk = 990;
    elevator1Config.Slot0.kP = Constants.Elevator.kP;
    elevator1Config.Slot0.kI = Constants.Elevator.kI;
    elevator1Config.Slot0.kD = Constants.Elevator.kD;
    elevator1Config.Slot0.kG = Constants.Elevator.kG;
    elevator1Config.Slot0.kA = Constants.Elevator.kA;
    elevator1Config.Slot0.kV = Constants.Elevator.kV;
    elevator2Config.Slot0.kP = Constants.Elevator.kP;
    elevator2Config.Slot0.kI = Constants.Elevator.kI;
    elevator2Config.Slot0.kD = Constants.Elevator.kD;
    elevator2Config.Slot0.kG = Constants.Elevator.kG;
    elevator2Config.Slot0.kA = Constants.Elevator.kA;
    elevator2Config.Slot0.kV = Constants.Elevator.kV;
    elevator1Config.CurrentLimits.StatorCurrentLimitEnable = true;
    elevator1Config.CurrentLimits.StatorCurrentLimit = 120;
    elevator1Config.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevator1Config.CurrentLimits.SupplyCurrentLimit = 50;
    elevator2Config.CurrentLimits.StatorCurrentLimitEnable = true;
    elevator2Config.CurrentLimits.StatorCurrentLimit = 120;
    elevator2Config.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevator2Config.CurrentLimits.SupplyCurrentLimit = 50;
    elevator1Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevator1Config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.Elevator.elevatorForwardSoftLimitRotations;
    elevator1Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevator1Config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.Elevator.elevatorReverseSoftLimitRotations;
    elevator2Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevator2Config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.Elevator.elevatorForwardSoftLimitRotations;
    elevator2Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevator2Config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.Elevator.elevatorReverseSoftLimitRotations;
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
            Constants.Elevator.kS, Constants.Elevator.kG, Constants.Elevator.kV);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var elevator1Status =
        StatusSignal.refreshAll(motor1Position, motor1Velocity, motor1AppliedVolts, motor1Current);
    var elevator2Status =
        StatusSignal.refreshAll(motor2Position, motor2Velocity, motor2AppliedVolts, motor2Current);

    inputs.elevator1Connected = motor1ConnectedDebounce.calculate(elevator1.isConnected());
    inputs.elevator1PositionMeters =
        motor1Position.getValueAsDouble() * Constants.Elevator.rotationsToMetersRatio;
    inputs.elevator1VelocityMPS =
        motor1Velocity.getValueAsDouble() * Constants.Elevator.rotationsToMetersRatio;
    inputs.elevator1AppliedVolts = motor1AppliedVolts.getValueAsDouble();
    inputs.elevator1CurrentAmps = motor1Current.getValueAsDouble();

    inputs.elevator2Connected = motor2ConnectedDebounce.calculate(elevator2.isConnected());
    inputs.elevator2PositionMeters =
        motor2Position.getValueAsDouble() * Constants.Elevator.rotationsToMetersRatio;
    inputs.elevator2VelocityMPS =
        motor2Velocity.getValueAsDouble() * Constants.Elevator.rotationsToMetersRatio;
    inputs.elevator2AppliedVolts = motor2AppliedVolts.getValueAsDouble();
    inputs.elevator2CurrentAmps = motor2Current.getValueAsDouble();
  }

  @Override
  public void setHeightClosedLoop(double meters) {
    // double voltage =
    // elevatorFeedforward.calculate(motor1Velocity.getValueAsDouble());
    // elevatorController.FeedForward = voltage;
    if (!isOverriding()) {
      elevatorController.Position = meters; // / Constants.Elevator.rotationsToMetersRatio;
      elevator2Controller.Position = meters; // / Constants.Elevator.rotationsToMetersRatio;
      elevatorController.FeedForward = elevatorFeedforward.calculate(0);
      elevatorController.FeedForward = elevatorFeedforward.calculate(0);
      elevator1.setControl(elevatorController);
      elevator2.setControl(elevator2Controller);
    }
  }

  private boolean overriding;

  public boolean isOverriding() {
    return overriding;
  }

  @Override
  public void setIsOverriding(boolean overriding) {
    this.overriding = overriding;
  }

  @Override
  public void setHeightClosedLoopOverride(double meters) {
    // double voltage =
    // elevatorFeedforward.calculate(motor1Velocity.getValueAsDouble());
    // elevatorController.FeedForward = voltage;
    elevatorController.Position = meters; // / Constants.Elevator.rotationsToMetersRatio;
    elevator2Controller.Position = meters; // / Constants.Elevator.rotationsToMetersRatio;
    elevatorController.FeedForward = elevatorFeedforward.calculate(0);
    elevatorController.FeedForward = elevatorFeedforward.calculate(0);
    elevator1.setControl(elevatorController);
    elevator2.setControl(elevator2Controller);
  }

  public void runCharacterization(double voltage) {
    elevator1.setVoltage(voltage);
    elevator2.setVoltage(voltage);
  }

  public void zeroMotors() {
    elevator1.setPosition(0);
    elevator2.setPosition(0);
  }

  public double getTarget() {
    return elevatorController.Position;
  }
}
