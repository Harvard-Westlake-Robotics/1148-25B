package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class WristConstants {
  // Motor constants
  public static final int motorId = 16;
  public static final String motorCANBusName = "rio";
  public static final InvertedValue motorInverted = InvertedValue.CounterClockwise_Positive;
  public static final int statorLimit = 120;
  public static final int supplyLimit = 60;

  // PID constants
  public static final double kP = 8;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.4;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double kG = 0.2;

  // Motion magic constants
  public static final double motionMagicAcceleration = 4;
  public static final double motionMagicCruiseVelocity = 2;
  public static final double motionMagicJerk = 1;

  // Physical constants
  public static final double motorRotationsPerWristRotationRatio = 36.73;
  public static final Angle angleOffset = Rotations.of(0.3218);
  public static final Angle wristMinAngle = Rotations.of(-0.1987);
  public static final Angle wristMaxAngle = Rotations.of(0.3213);
  public static final Distance wristLength = Meters.of(0.10);
}
