package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.LinearVelocity;

public class HangConstants {
  // Motor constants
  public static final int motorId = 20;
  public static final InvertedValue motorInverted = InvertedValue.CounterClockwise_Positive;

  // PID constants
  public static final double kP = 10;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.1;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  // Motion magic constants; we are using velocity so these don't matter
  public static final double motionMagicAcceleration = 9999;
  public static final double motionMagicCruiseVelocity = 9999;

  // Velocity constants
  public static final LinearVelocity hangVelocity = MetersPerSecond.of(15);

  // Physical constants
  public static final double rotationsPerMeterRatio =
      3.0 / (4.0 * Math.PI); // 3:1 reduction and 4" diameter wheels
}
