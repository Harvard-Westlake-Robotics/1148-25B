package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.pivot.Pivot;

public class ElevatorIOTalonFX implements ElevatorIO {
  // Motors and elevator controllers
  private final TalonFX elevatorMotor1;
  private final TalonFX elevatorMotor2;
  private final MotionMagicVoltage elevatorController;

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  public ElevatorIOTalonFX() {
    elevatorMotor1 = new TalonFX(ElevatorConstants.elevator1ID);
    elevatorMotor2 = new TalonFX(ElevatorConstants.elevator2ID);
    elevatorMotor1.setPosition(0);
    elevatorMotor2.setPosition(0);
    elevatorController = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.MotorOutput.Inverted = ElevatorConstants.elevatorInverted;

    elevatorConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.motionMagicAcceleration;
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.motionMagicCruiseVelocity;
    elevatorConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.motionMagicJerk;

    elevatorConfig.Slot0.kP = ElevatorConstants.kP;
    elevatorConfig.Slot0.kI = ElevatorConstants.kI;
    elevatorConfig.Slot0.kD = ElevatorConstants.kD;
    elevatorConfig.Slot0.kS = ElevatorConstants.kS;
    elevatorConfig.Slot0.kV = ElevatorConstants.kV;
    elevatorConfig.Slot0.kG = ElevatorConstants.kG;
    elevatorConfig.Slot0.kA = ElevatorConstants.kA;

    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.statorLimit;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.supplyLimit;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.elevatorForwardSoftLimitRotations;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ElevatorConstants.elevatorReverseSoftLimitRotations;

    elevatorMotor1.getConfigurator().apply(elevatorConfig);
    elevatorMotor1.setControl(elevatorController);

    elevatorMotor2.getConfigurator().apply(elevatorConfig);
    elevatorMotor2.setControl(elevatorController);

    motorPosition = elevatorMotor1.getPosition();
    motorVelocity = elevatorMotor1.getVelocity();
    motorAppliedVolts = elevatorMotor1.getMotorVoltage();
    motorCurrent = elevatorMotor1.getStatorCurrent();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    StatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.elevatorConnected = motorConnectedDebouncer.calculate(elevatorMotor1.isConnected());
    inputs.elevatorPositionMeters =
        motorPosition.getValueAsDouble() / ElevatorConstants.rotationsPerMeterRatio;
    inputs.elevatorVelocityMPS =
        motorVelocity.getValueAsDouble() / ElevatorConstants.rotationsPerMeterRatio;
    inputs.elevatorAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.elevatorCurrentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void runVoltage(double voltage) {
    elevatorMotor1.setControl(new VoltageOut(voltage));
    elevatorMotor2.setControl(new VoltageOut(voltage));
  }

  @Override
  public void goToHeightClosedLoop(double height) {
    elevatorMotor1.setControl(
        elevatorController
            .withPosition(height * ElevatorConstants.rotationsPerMeterRatio)
            .withFeedForward(
                Math.cos(Units.rotationsToRadians(Pivot.getInstance().getAngleRots()))));
    elevatorMotor2.setControl(
        elevatorController
            .withPosition(height * ElevatorConstants.rotationsPerMeterRatio)
            .withFeedForward(
                Math.cos(Units.rotationsToRadians(Pivot.getInstance().getAngleRots()))));
  }

  @Override
  public void tareHeight(double height) {
    elevatorMotor1.setPosition(height * ElevatorConstants.rotationsPerMeterRatio);
    elevatorMotor2.setPosition(height * ElevatorConstants.rotationsPerMeterRatio);
  }

  @Override
  public double getTarget() {
    return elevatorController.Position;
  }
}
