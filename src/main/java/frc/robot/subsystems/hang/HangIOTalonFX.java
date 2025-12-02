package frc.robot.subsystems.hang;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.HangConstants;

public class HangIOTalonFX implements HangIO {
  private final TalonFX hangMotor;
  private final MotionMagicVelocityVoltage hangController;

  // Actually in meters
  private final StatusSignal<Angle> motorPosition;
  // Actually in m/s
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  public HangIOTalonFX() {
    hangMotor = new TalonFX(HangConstants.motorId);
    hangMotor.setPosition(0);
    hangController = new MotionMagicVelocityVoltage(RotationsPerSecond.of(0)).withEnableFOC(true);

    TalonFXConfiguration hangConfig = new TalonFXConfiguration();

    hangConfig.MotorOutput.Inverted = HangConstants.motorInverted;
    hangConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    hangConfig.CurrentLimits.StatorCurrentLimit = HangConstants.statorLimit.in(Amps);
    hangConfig.CurrentLimits.SupplyCurrentLimit = HangConstants.supplyLimit.in(Amps);

    hangConfig.Slot0.kP = HangConstants.kP;
    hangConfig.Slot0.kI = HangConstants.kI;
    hangConfig.Slot0.kD = HangConstants.kD;
    hangConfig.Slot0.kS = HangConstants.kS;
    hangConfig.Slot0.kV = HangConstants.kV;
    hangConfig.Slot0.kA = HangConstants.kA;

    hangConfig.MotionMagic.MotionMagicAcceleration = HangConstants.motionMagicAcceleration;
    hangConfig.MotionMagic.MotionMagicCruiseVelocity = HangConstants.motionMagicCruiseVelocity;
    hangMotor.getConfigurator().apply(hangConfig);
    hangMotor.setControl(hangController);

    motorPosition = hangMotor.getPosition();
    motorVelocity = hangMotor.getVelocity();
    motorAppliedVolts = hangMotor.getMotorVoltage();
    motorCurrent = hangMotor.getStatorCurrent();
  }

  @Override
  public void updateInputs(HangIOInputs inputs) {
    StatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.motorConnected = motorConnectedDebouncer.calculate(hangMotor.isConnected());
    inputs.motorPosition =
        Meters.of(motorPosition.getValue().in(Rotations) / HangConstants.rotationsPerMeterRatio);
    inputs.motorVelocity =
        MetersPerSecond.of(motorVelocity.getValue().in(RotationsPerSecond) / HangConstants.rotationsPerMeterRatio);
    inputs.motorAppliedVoltage = motorAppliedVolts.getValue();
    inputs.motorCurrent = motorCurrent.getValue();
  }

  @Override
  public void runVoltage(Voltage voltage) {
    hangMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void runVelocityClosedLoop(LinearVelocity velocity) {
    hangMotor.setControl(
        hangController.withVelocity(
            velocity.in(MetersPerSecond) * HangConstants.rotationsPerMeterRatio));
  }
}
