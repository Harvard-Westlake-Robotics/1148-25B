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
    public double motorCurrentAmps = 0.0;
  }

  public default void updateInputs(HangIOInputs inputs) {}
  
  public default void runCharacterization(double volts) {}

  public default void runVelocity(LinearVelocity velocity) {}
}
