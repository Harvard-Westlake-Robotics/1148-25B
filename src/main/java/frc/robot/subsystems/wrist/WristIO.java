package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean wristMotorConnected = false;
    public Angle wristAngle = Degrees.of(0.0);
    public AngularVelocity wristVelocity = DegreesPerSecond.of(0.0);
    public Voltage wristAppliedVoltage = Volts.of(0.0);
    public Current wristCurrent = Amps.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  public default void runVoltage(Voltage voltage) {}

  public default void goToAngleClosedLoop(Angle wristAngle) {}

  public default void tareAngle(Angle wristAngle) {}

  public default Angle getTarget() {
    return Degrees.of(0.0);
  }
}
