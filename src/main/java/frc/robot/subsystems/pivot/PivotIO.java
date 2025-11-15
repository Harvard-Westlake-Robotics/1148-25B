package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public boolean pivotMotor1Connected = false;
    public boolean pivotMotor2Connected = false;
    public boolean pivotMotor3Connected = false;
    public boolean pivotEncoderConnected = false;
    public Angle pivotAngle = Degrees.of(0.0);
    public AngularVelocity pivotVelocity = DegreesPerSecond.of(0.0);
    public Voltage pivotAppliedVoltage = Volts.of(0.0);
    public Current pivotCurrent = Amps.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  public default void runVoltage(Voltage voltage) {}

  public default void goToAngleClosedLoop(Angle pivotAngle) {}

  public default void tareAngle(Angle pivotAngle) {}

  public default void setTunableConstants(
      double kP,
      double kI,
      double kD,
      Voltage kS,
      Per<VoltageUnit, AngularVelocityUnit> kV,
      Voltage kG,
      Per<VoltageUnit, AngularAccelerationUnit> kA,
      AngularAcceleration motionMagicAcceleration,
      AngularVelocity motionMagicCruiseVelocity,
      Velocity<AngularAccelerationUnit> motionMagicJerk,
      Angle pivotAngle) {}

  public default Angle getPivotTarget() {
    return Rotations.of(0.0);
  }
}
