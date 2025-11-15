package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorConstants {
  // Motor constants
  public static final int elevator1ID = 18;
  public static final int elevator2ID = 19;
  public static final InvertedValue elevatorInverted = InvertedValue.Clockwise_Positive;
  public static final Current statorLimit = Amps.of(120);
  public static final Current supplyLimit = Amps.of(50);

  // PID constants
  public static double kP = 1.5;
  public static double kI = 0.0;
  public static double kD = 0.3;
  public static Voltage kS = Volts.of(0.0);
  public static Per<VoltageUnit, LinearVelocityUnit> kV = VoltsPerMeterPerSecond.ofNative(0.0);
  public static Voltage kG = Volts.of(0.0);
  public static Per<VoltageUnit, LinearAccelerationUnit> kA = VoltsPerMeterPerSecondSquared.ofNative(0.0);

  // Motion magic constants
  public static final LinearAcceleration motionMagicAcceleration = MetersPerSecondPerSecond.of(2.5);
  public static final LinearVelocity motionMagicCruiseVelocity = MetersPerSecond.of(1);
  public static final Velocity<LinearAccelerationUnit> motionMagicJerk = MetersPerSecondPerSecond.per(Second).of(5);

  // Physical constants
  public static final Distance elevatorForwardSoftLimit = Inches.of(42);
  public static final Distance elevatorReverseSoftLimit = Inches.of(0);
  public static final double rotationsPerMeterRatio = 35.1519; // 1 / (1.12 / 39.3701)
  public static final Distance armMinLength = Inches.of(24.654);
  public static final Distance armMaxLength = Inches.of(67.056);
}
