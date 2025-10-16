package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;

public class WristConstants {
  // Motor constants
  public final int motorId;
  public final int motorId2;
  public final int motorId3;
  public final InvertedValue motorInverted;
  public final int statorLimit;
  public final int supplyLimit;

  // PID constants
  public double kP;
  public double kI;
  public double kD;
  public double kS;
  public double kV;
  public double kG;
  public double kA;

  // Motion magic constants
  public final double motionMagicAcceleration;
  public final double motionMagicCruiseVelocity;
  public final double motionMagicJerk;

  // Velocity constants
  public final double wristVelocity;

  // Physical constants
  public final double motorToWristRotations;
  public final Angle angleOffset;
  public final Angle wristMinAngle;
  public final Angle wristMaxAngle;

  public WristConstants(int motorId, int motorId2, int motorId3, InvertedValue motorInverted, int statorLimit,
      int supplyLimit, double kP, double kI, double kD, double kS, double kV, double kG, double kA,
      double motionMagicAcceleration, double motionMagicCruiseVelocity, double motionMagicJerk, double wristVelocity,
      double motorToWristRotations, Angle angleOffset, Angle wristMinAngle, Angle wristMaxAngle) {
    this.motorId = motorId;
    this.motorId2 = motorId2;
    this.motorId3 = motorId3;
    this.motorInverted = motorInverted;
    this.statorLimit = statorLimit;
    this.supplyLimit = supplyLimit;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kV = kV;
    this.kG = kG;
    this.kA = kA;
    this.motionMagicAcceleration = motionMagicAcceleration;
    this.motionMagicCruiseVelocity = motionMagicCruiseVelocity;
    this.motionMagicJerk = motionMagicJerk;
    this.wristVelocity = wristVelocity;
    this.motorToWristRotations = motorToWristRotations;
    this.angleOffset = angleOffset;
    this.wristMinAngle = wristMinAngle;
    this.wristMaxAngle = wristMaxAngle;
  }

  // TODO: Fix later with real values
  public static final WristConstants ShoulderWrist = new WristConstants(
      18,
      19,
      20,
      InvertedValue.CounterClockwise_Positive,
      40,
      80,
      5,
      0.0,
      0.0,
      0.000,
      0.0,
      0.0,
      0.0,
      1000.0,
      1000.0,
      1000.0,
      100000,
      1.0, // 4.846
      Angle.ofBaseUnits(0.0, Degrees), // AngleOffset
      Angle.ofBaseUnits(0, Rotations), // Resting on base
      Angle.ofBaseUnits(0.569, Rotations)); // 205 deg

  // TODO: Fix later with real values
  public static final WristConstants IntakeWrist = new WristConstants(
      0,
      0,
      0,
      InvertedValue.CounterClockwise_Positive,
      40,
      80,
      5,
      0.0,
      0.0,
      0.000,
      0.0,
      0.0,
      0.0,
      1000.0,
      1000.0,
      1000.0,
      10,
      1.0,
      Angle.ofBaseUnits(0.0, Degrees), // AngleOffset
      Angle.ofBaseUnits(-1.0 / 3.0, Rotations), // -120°
      Angle.ofBaseUnits(1.0 / 3.0, Rotations)); // +120°
}
