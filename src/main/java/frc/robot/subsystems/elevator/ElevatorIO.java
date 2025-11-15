package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean elevatorConnected = false;
    public Distance elevatorHeight = Meters.of(0.0);
    public LinearVelocity elevatorVelocity = MetersPerSecond.of(0.0);
    public Voltage elevatorAppliedVoltage = Volts.of(0.0);
    public Current elevatorCurrent = Amps.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void runVoltage(Voltage voltage) {}

  public default void goToHeightClosedLoop(Distance height) {}

  /** Sets the motor's internally stored position to what it is fed */
  public default void tareHeight(Distance height) {}

  public default Distance getTarget() {
    return Meters.of(0.0);
  }
}
