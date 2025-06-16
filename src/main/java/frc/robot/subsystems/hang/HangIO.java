package frc.robot.subsystems.hang;

import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface HangIO {
  @AutoLog
  public static class HangIOInputs {
    public boolean motorConnected = false;
    public double motorPositionMeters = 0.0;
    public double motorVelocityMPS = 0.0;
    public double motorAppliedVolts = 0.0;
    public double motorCurrent = 0.0;
  }

  public default void updateInputs(HangIOInputs inputs) {}

  public default void runVelocity(LinearVelocity velocity) {}

  public default void runCharacterization(double volts) {}
}
