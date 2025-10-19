package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class WristConstants {
  // Motor constants
  // ID of the first motor
  public final int motorId;
  public final InvertedValue motorInverted;
  public final int statorLimit;
  public final int supplyLimit;

  // PID constants
  public double kP;
  public double kI;
  public double kD;
  public double kS;
  public double kV;
  public double kA;
  // Determine kG by gradually increasing phoenix tuner voltage until the wrist can hold itself up
  // (velocity is 0), then divide the voltage by the cosine of the absolute angle to get kG
  public double kG;

  // Motion magic constants
  public final double motionMagicAcceleration;
  public final double motionMagicCruiseVelocity;
  public final double motionMagicJerk;

  // Physical constants
  public final double motorRotationsPerWristRotationRatio;
  public final Angle angleOffset;
  public final Angle wristMinAngle;
  public final Angle wristMaxAngle;
  public final Distance wristLength;

  public WristConstants(
      int motorId,
      InvertedValue motorInverted,
      int statorLimit,
      int supplyLimit,
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double motionMagicAcceleration,
      double motionMagicCruiseVelocity,
      double motionMagicJerk,
      double motorRotationsPerWristRotationRatio,
      Angle angleOffset,
      Angle wristMinAngle,
      Angle wristMaxAngle,
      Distance wristLength) {
    this.motorId = motorId;
    this.motorInverted = motorInverted;
    this.statorLimit = statorLimit;
    this.supplyLimit = supplyLimit;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.kG = kG;
    this.motionMagicAcceleration = motionMagicAcceleration;
    this.motionMagicCruiseVelocity = motionMagicCruiseVelocity;
    this.motionMagicJerk = motionMagicJerk;
    this.motorRotationsPerWristRotationRatio = motorRotationsPerWristRotationRatio;
    this.angleOffset = angleOffset;
    this.wristMinAngle = wristMinAngle;
    this.wristMaxAngle = wristMaxAngle;
    this.wristLength = wristLength;
  }

  // Pivot encoder constants
  public static final int pivotEncoderId = 30;
  public static final double pivotEncoderOffset = 0;
  public static final SensorDirectionValue pivotEncoderSensorDirection =
      SensorDirectionValue.CounterClockwise_Positive;

  // TODO: Fix later with real values
  public static final WristConstants Pivot =
      new WristConstants(
          25,
          InvertedValue.Clockwise_Positive,
          40,
          80,
          0.2,
          0.0,
          0.0,
          0.1,
          0.0,
          0.0,
          0.0,
          0.5 * 112.5, // 0.5 rot/s^2
          0.2 * 112.5, // 0.2 rot/s
          1 * 112.5, // 1 rot/s^3
          112.5,
          Degrees.of(0.0), // AngleOffset
          Rotations.of(0), // Resting on base
          Degrees.of(205), // 205 deg
          Meters.of(0)); // Unused

  // TODO: Fix later with real values
  public static final WristConstants Wrist =
      new WristConstants(
          16,
          InvertedValue.CounterClockwise_Positive,
          40,
          80,
          5,
          0.0,
          0.0,
          0.1,
          0.0,
          0.0,
          0.0,
          1000.0,
          1000.0,
          1000.0,
          36.73,
          Rotations.of(11.98),
          Degrees.of(-100),
          Degrees.of(72.5),
          Meters.of(0.10));
}
