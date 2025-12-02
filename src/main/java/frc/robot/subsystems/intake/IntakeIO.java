package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeMotorConnected = false;
    public Distance intakeMotorPosition = Meters.of(0.0);
    public LinearVelocity intakeMotorVelocity = MetersPerSecond.of(0.0);
    public Voltage intakeMotorAppliedVoltage = Volts.of(0.0);
    public Current intakeMotorCurrent = Amps.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runVoltage(Voltage voltage) {}

  public default void runVelocity(LinearVelocity velocity) {}

  public Boolean getSensor1();

  public Boolean getSensor2();

  public Boolean getSensor3();

  public Boolean getSensor4();
}
