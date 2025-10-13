package frc.robot.subsystems.hang;

import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface HangIO {
  @AutoLog
  /*
   * Makes the instance variables and assigns values to them
   */
  public static class HangIOInputs {
    public boolean motorConnected = false;
    public double motorPositionMeters = 0.0;
    public double motorVelocityMPS = 0.0;
    public double motorAppliedVolts = 0.0;
    public double motorCurrent = 0.0;
  }

  // Updates the inputs
  public default void updateInputs(HangIOInputs inputs) {}

  // Runs the hang at the velocity
  public default void runVelocity(LinearVelocity velocity) {}

  // Runs the hang with the characterization
  public default void runCharacterization(double volts) {}
}
