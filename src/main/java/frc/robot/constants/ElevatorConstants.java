package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
  // Motor constants
  public static final int elevator1ID = 13;
  public static final int elevator2ID = 14;
  public static final InvertedValue elevator2Inverted = InvertedValue.Clockwise_Positive;
  public static final InvertedValue elevator1Inverted = InvertedValue.Clockwise_Positive;
  public static final int statorLimit = 120;
  public static final int supplyLimit = 50;

  // PID constants
  public static double kP = 5.0;
  public static double kI = 0.0;
  public static double kD = 0.0;
  public static double kS = 0.1;
  public static double kV = 0.0;
  public static double kG = 0.0;
  public static double kA = 0.0;

  // Motion magic constants
  public static final double motionMagicAcceleration = 390;
  public static final double motionMagicCruiseVelocity = 250;
  public static final double motionMagicJerk = 990;

  // Physical constants
  public static final double elevatorForwardSoftLimitRotations = 55;
  public static final double elevatorReverseSoftLimitRotations = 0.0;
  public static final double rotationsToMetersRatio = (0.028448);
  public static final double elevatorGroundOffsetMeters = 0.2125;
  public static final Distance armMinLength = Meters.of(0.0);
  public static final Distance armMaxLength = Meters.of(42.5);
}
