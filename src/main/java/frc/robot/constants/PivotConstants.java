package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;

public class PivotConstants {
  // Motor constants
  // ID of the first motor
  public static final int motorId = 25;
  public static final InvertedValue motorInverted = InvertedValue.Clockwise_Positive;
  public static final int statorLimit = 40;
  public static final int supplyLimit = 80;

  // PID constants
  public static double kP = 8;
  public static double kI = 0.0;
  public static double kD = 0.0;
  public static double kS = 0.0;
  public static double kV = 0.0;
  public static double kA = 0.0;
  // Determine kG by gradually increasing phoenix tuner voltage until the Pivot can hold itself up
  // (velocity is 0), then divide the voltage by the cosine of the absolute angle to get kG
  public static double kG = 0;

  // Motion magic constants
  public static final double motionMagicAcceleration = 0.25 * 112.5; // 0.5 rot/s^2
  public static final double motionMagicCruiseVelocity = 0.3 * 112.5; // 0.2 rot/s
  public static final double motionMagicJerk = 1 * 112.5; // 1 rot/s^3

  // Physical constants
  public static final double motorRotationsPerPivotRotationRatio = 112.5;
  public static final Angle angleOffset = Rotations.of(20.43); // AngleOffset
  public static final Angle pivotMinAngle = Rotations.of(0); // Resting on base
  public static final Angle pivotMaxAngle = Rotations.of(0.29);

  // Pivot encoder constants
  public static final int pivotEncoderId = 30;
  public static final double pivotEncoderOffset = 0.403;
  public static final SensorDirectionValue pivotEncoderSensorDirection =
      SensorDirectionValue.Clockwise_Positive;
}
