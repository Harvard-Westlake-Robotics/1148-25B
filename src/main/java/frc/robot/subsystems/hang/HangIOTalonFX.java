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
import frc.robot.constants.Constants;

public class HangIOTalonFX implements HangIO {
  private TalonFX hangMotor;
  private MotionMagicVelocityTorqueCurrentFOC hangController;
  private VoltageOut voltageRequest = new VoltageOut(0);

  private TalonFXConfiguration hangConfig;

  private SimpleMotorFeedforward hangFeedforward;

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Connection debouncers
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  public HangIOTalonFX() {
    hangMotor = new TalonFX(Constants.Hang.motorId);
    hangMotor.setPosition(0);
    hangController =
        new MotionMagicVelocityTorqueCurrentFOC(
            AngularVelocity.ofBaseUnits(0.0, RotationsPerSecond));

    TalonFXConfiguration hangConfig = new TalonFXConfiguration();

    hangConfig.MotorOutput.Inverted = Constants.Hang.motorInverted;
    hangConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    hangConfig.Slot0.kP = Constants.Hang.kP;
    hangConfig.Slot0.kI = Constants.Hang.kI;
    hangConfig.Slot0.kD = Constants.Hang.kD;
    hangConfig.Slot0.kS = Constants.Hang.kS;
    hangConfig.Slot0.kA = Constants.Hang.kA;
    hangConfig.Slot0.kV = Constants.Hang.kV;
    hangMotor.getConfigurator().apply(hangConfig);
    this.hangConfig = hangConfig;
    hangMotor.setControl(hangController);

    motorPosition = hangMotor.getPosition();
    motorVelocity = hangMotor.getVelocity();
    motorAppliedVolts = hangMotor.getMotorVoltage();
    motorCurrent = hangMotor.getStatorCurrent();
  }

  @Override
  public void updateInputs(HangIOInputs inputs) {
    var hangStatus =
        StatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.motorConnected = motorConnectedDebounce.calculate(hangMotor.isConnected());
    inputs.motorPositionMeters =
        motorPosition.getValueAsDouble() * Constants.Hang.rotationsToMetersRatio;
    inputs.motorVelocityMPS =
        motorVelocity.getValueAsDouble() * Constants.Hang.rotationsToMetersRatio;
    inputs.motorAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.motorCurrent = motorCurrent.getValueAsDouble();
  }

  @Override
  public void runVelocity(LinearVelocity velocity) {
    hangConfig.MotionMagic.MotionMagicCruiseVelocity = 9999;
    hangConfig.MotionMagic.MotionMagicAcceleration = 9999;
    hangMotor.getConfigurator().apply(hangConfig);
    hangController.FeedForward =
        hangFeedforward.calculate(
            velocity.in(MetersPerSecond) / Constants.Hang.rotationsToMetersRatio);
    hangController.Velocity = velocity.in(MetersPerSecond) / Constants.Hang.rotationsToMetersRatio;
    hangMotor.setControl(hangController);
  }

  @Override
  public void runCharacterization(double voltage) {
    hangMotor.setControl(new VoltageOut(voltage));
  }
}
