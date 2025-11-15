package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.pivot.Pivot;

public class ElevatorIOTalonFX implements ElevatorIO {
  // Motors and elevator controllers
  private final TalonFX elevatorMotor1;
  private final TalonFX elevatorMotor2;
  private final MotionMagicVoltage elevatorController;

  // Actually in meters
  private final StatusSignal<Angle> elevatorPosition;
  // Actually in m/s
  private final StatusSignal<AngularVelocity> elevatorVelocity;
  private final StatusSignal<Voltage> motorAppliedVoltage;
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

    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.motionMagicAcceleration.in(MetersPerSecondPerSecond);
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.motionMagicCruiseVelocity.in(MetersPerSecond);
    elevatorConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.motionMagicJerk.in(MetersPerSecondPerSecond.per(Second));

    elevatorConfig.Feedback.RotorToSensorRatio = 1.0;
    elevatorConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.rotationsPerMeterRatio;

    elevatorConfig.Slot0.kP = ElevatorConstants.kP;
    elevatorConfig.Slot0.kI = ElevatorConstants.kI;
    elevatorConfig.Slot0.kD = ElevatorConstants.kD;
    elevatorConfig.Slot0.kS = ElevatorConstants.kS.in(Volts);
    elevatorConfig.Slot0.kV = ElevatorConstants.kV.in(VoltsPerMeterPerSecond);
    elevatorConfig.Slot0.kG = ElevatorConstants.kG.in(Volts);
    elevatorConfig.Slot0.kA = ElevatorConstants.kA.in(VoltsPerMeterPerSecondSquared);

    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.statorLimit.in(Amps);
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.supplyLimit.in(Amps);
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.elevatorForwardSoftLimit.in(Meters);
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ElevatorConstants.elevatorReverseSoftLimit.in(Meters);

    elevatorMotor1.getConfigurator().apply(elevatorConfig);
    elevatorMotor1.setControl(elevatorController);

    elevatorMotor2.getConfigurator().apply(elevatorConfig);
    elevatorMotor2.setControl(elevatorController);

    elevatorPosition = elevatorMotor1.getPosition();
    elevatorVelocity = elevatorMotor1.getVelocity();
    motorAppliedVoltage = elevatorMotor1.getMotorVoltage();
    motorCurrent = elevatorMotor1.getStatorCurrent();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    StatusSignal.refreshAll(elevatorPosition, elevatorVelocity, motorAppliedVoltage, motorCurrent);

    inputs.elevatorConnected = motorConnectedDebouncer.calculate(elevatorMotor1.isConnected());
    // Phoenix6 requires motor positions/velocities to be in angle units, so they must be cast to distance units here
    inputs.elevatorHeight = Meters.of(elevatorPosition.getValue().in(Rotations));
    inputs.elevatorVelocity = MetersPerSecond.of(elevatorVelocity.getValue().in(RotationsPerSecond));
    inputs.elevatorAppliedVoltage = motorAppliedVoltage.getValue();
    inputs.elevatorCurrent = motorCurrent.getValue();
  }

  @Override
  public void runVoltage(Voltage voltage) {
    elevatorMotor1.setControl(new VoltageOut(voltage));
    elevatorMotor2.setControl(new VoltageOut(voltage));
  }

  @Override
  public void goToHeightClosedLoop(Distance height) {
    elevatorMotor1.setControl(
        elevatorController
            .withPosition(height.in(Meters))
            .withFeedForward(
                Math.cos(Pivot.getInstance().getAngle().in(Radians))));
    elevatorMotor2.setControl(
        elevatorController
            .withPosition(height.in(Meters))
            .withFeedForward(
                Math.cos(Pivot.getInstance().getAngle().in(Radians))));
  }

  @Override
  public void tareHeight(Distance height) {
    elevatorMotor1.setPosition(height.in(Meters));
    elevatorMotor2.setPosition(height.in(Meters));
  }

  @Override
  public Distance getTarget() {
    return Meters.of(elevatorController.Position);
  }
}
