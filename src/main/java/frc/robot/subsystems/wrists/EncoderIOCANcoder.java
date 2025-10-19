package frc.robot.subsystems.wrists;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.WristConstants;

public class EncoderIOCANcoder implements EncoderIO {
  private CANcoder cancoder;
  private CANcoderConfiguration encoderConfig;

  private final StatusSignal<Angle> encoderAngle;
  private final StatusSignal<AngularVelocity> encoderVelocity;

  private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5);

  public EncoderIOCANcoder() {
    cancoder = new CANcoder(WristConstants.pivotEncoderId);

    encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = WristConstants.pivotEncoderOffset;
    encoderConfig.MagnetSensor.SensorDirection = WristConstants.pivotEncoderSensorDirection;
    cancoder.getConfigurator().apply(encoderConfig);

    encoderAngle = cancoder.getPosition();
    encoderVelocity = cancoder.getVelocity();
  }

  @Override
  public void updateInputs(EncoderIOInputs inputs) {
    StatusSignal.refreshAll(encoderAngle, encoderVelocity);

    inputs.encoderConnected = encoderConnectedDebouncer.calculate(cancoder.isConnected());
    inputs.encoderPositionRot = cancoder.getPosition().getValueAsDouble();
    inputs.encoderVelocityRPS = cancoder.getVelocity().getValueAsDouble();
  }

  @Override
  public double getAngle() {
    // 1:1 ratio
    return cancoder.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public double getAngularVelocity() {
    return cancoder.getVelocity().getValueAsDouble();
  }
}
