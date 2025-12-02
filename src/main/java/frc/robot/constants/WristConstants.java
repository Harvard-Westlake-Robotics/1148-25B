package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;

public class WristConstants {
  // Motor constants
  public static final int motorId = 16;
  public static final String motorCANBusName = "rio";
  public static final InvertedValue motorInverted = InvertedValue.CounterClockwise_Positive;
  public static final Current statorLimit = Amps.of(120);
  public static final Current supplyLimit = Amps.of(60);

  // PID constants
  public static final double kP = 8;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.4;
  public static final double kV = 0.0;
  public static final double kG = 0.2;
  public static final double kA = 0.0;

  // Motion magic constants
  public static final AngularAcceleration motionMagicAcceleration = RotationsPerSecondPerSecond.of(0.0);
  public static final AngularVelocity motionMagicCruiseVelocity = RotationsPerSecond.of(0.0);
  public static final Velocity<AngularAccelerationUnit> motionMagicJerk = RotationsPerSecondPerSecond.per(Second).of(0.0);

  // Physical constants
  public static final double motorRotationsPerWristRotationRatio = 36.73;
  public static final Angle angleOffset = Rotations.of(0.3218);
  public static final Angle wristMinAngle = Rotations.of(-0.1987);
  public static final Angle wristMaxAngle = Rotations.of(0.3213);
  public static final Distance wristLength = Meters.of(0.10);
}
