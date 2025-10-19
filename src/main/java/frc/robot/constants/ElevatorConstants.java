package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
  // Motor constants
  public static final int elevator1ID = 18;
  public static final int elevator2ID = 19;
  public static final InvertedValue elevator2Inverted = InvertedValue.Clockwise_Positive;
  public static final InvertedValue elevator1Inverted = InvertedValue.Clockwise_Positive;
  public static final int statorLimit = 80;
  public static final int supplyLimit = 40;

  // PID constants
  // TODO: Tune
  public static double kP = 5.0;
  public static double kI = 0.0;
  public static double kD = 0.0;
  public static double kS = 0.1;
  public static double kV = 0.0;
  public static double kG = 0.0;
  public static double kA = 0.0;

  // Motion magic constants
  public static final double motionMagicAcceleration = 1757; // 50m/s^2 in rot/s^2
  public static final double motionMagicCruiseVelocity = 703; // 20m/s in rot/s
  public static final double motionMagicJerk = 3515; // 100 m/s^3 in rot/s^3

  // Physical constants
  public static final double elevatorForwardSoftLimitRotations = 38;
  public static final double elevatorReverseSoftLimitRotations = 0.0;
  public static final double rotationsPerMeterRatio = 35.1519; // 1 / (1.12 / 39.3701)
  public static final Distance armMinLength = Inches.of(24.654);
  public static final Distance armMaxLength = Inches.of(67.056);
}
