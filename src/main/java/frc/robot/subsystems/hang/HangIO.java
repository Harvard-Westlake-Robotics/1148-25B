package frc.robot.subsystems.hang;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

public interface HangIO {
  @AutoLog
  public static class HangIOInputs {
    public boolean motorConnected = false;
    public Distance motorPosition = Meters.of(0.0);
    public LinearVelocity motorVelocity = MetersPerSecond.of(0.0);
    public Voltage motorAppliedVoltage = Volts.of(0.0);
    public Current motorCurrent = Amps.of(0.0);
  }

  public default void updateInputs(HangIOInputs inputs) {}

  public default void runVoltage(Voltage voltage) {}

  public default void runVelocityClosedLoop(LinearVelocity velocity) {}
}
