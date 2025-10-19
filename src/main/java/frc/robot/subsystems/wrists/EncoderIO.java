package frc.robot.subsystems.wrists;

import org.littletonrobotics.junction.AutoLog;

public interface EncoderIO {
  @AutoLog
  public static class EncoderIOInputs {
    public boolean encoderConnected = false;
    public double encoderPositionRot = 0;
    public double encoderVelocityRPS = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(EncoderIOInputs inputs) {}

  public default double getAngle() {
    return 0;
  }

  public default double getAngularVelocity() {
    return 0;
  }
}
