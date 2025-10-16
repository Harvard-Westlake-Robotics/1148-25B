package frc.robot.subsystems.hang;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.HangConstants;

public class HangIOTalonFX implements HangIO {
  private TalonFX hangMotor;
  private MotionMagicVelocityTorqueCurrentFOC hangController;

  private SimpleMotorFeedforward hangFeedforward;

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  public HangIOTalonFX() {
    hangMotor = new TalonFX(HangConstants.motorId);
    hangMotor.setPosition(0);
    hangController =
        new MotionMagicVelocityTorqueCurrentFOC(
            AngularVelocity.ofBaseUnits(0.0, RotationsPerSecond));

    TalonFXConfiguration hangConfig = new TalonFXConfiguration();

    hangConfig.MotorOutput.Inverted = HangConstants.motorInverted;
    hangConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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

    hangFeedforward =
        new SimpleMotorFeedforward(HangConstants.kS, HangConstants.kV, HangConstants.kA);
  }

  public void updateInputs(HangIOInputs inputs) {
    StatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.motorConnected = motorConnectedDebounce.calculate(hangMotor.isConnected());
    inputs.motorPositionMeters =
        motorPosition.getValueAsDouble() * (1 / HangConstants.rotationsToMetersRatio);
    inputs.motorVelocityMPS =
        motorVelocity.getValueAsDouble() * (1 / HangConstants.rotationsToMetersRatio);
    inputs.motorAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.motorCurrent = motorCurrent.getValueAsDouble();
  }

  @Override
  public void runVelocity(LinearVelocity velocity) {
    hangController.FeedForward =
        hangFeedforward.calculate(
            velocity.in(MetersPerSecond) / HangConstants.rotationsToMetersRatio);
    hangController.Velocity = velocity.in(MetersPerSecond) / HangConstants.rotationsToMetersRatio;
    hangMotor.setControl(hangController);
  }

  @Override
  public void runCharacterization(double voltage) {
    hangMotor.setControl(new VoltageOut(voltage));
  }
}
