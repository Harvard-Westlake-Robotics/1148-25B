package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class WristConstants {
  public final int motorId;
  public final int motorId2;
  public final int motorId3;
  public final InvertedValue motorInverted;
  public final double wristVelocity;
  public double kP;
  public double kI;
  public double kD;
  public double kS;
  public double kV;
  public double kG;
  public double kA;

  public double ANGLE_MAX_ACCELERATION;
  public double ANGLE_MAX_VELOCITY;
  public double ANGLe_MAX_JERK;
  public double motorToWristRotations;
  public Angle angleOffset;
  public int statorLimit;
  public int supplyLimit;
  public Rotation2d WRIST_MIN_DEG;
  public Rotation2d WRIST_MAX_DEG;

  public WristConstants(
      int motorId,
      int motorId2,
      int motorId3,
      InvertedValue motorInverted,
      double wristVelocity,
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kG,
      double kA,
      double ANGLE_MAX_ACCELERATION,
      double ANGLE_MAX_VELOCITY,
      double ANGLe_MAX_JERK,
      double motorToWristRotations,
      Angle angleOffset,
      int supplyLimit,
      int statorLimit,
      Rotation2d WRIST_MIN_DEG,
      Rotation2d WRIST_MAX_DEG) {
    this.motorId = motorId;
    this.motorId2 = motorId2;
    this.motorId3 = motorId3;
    this.motorInverted = motorInverted;
    this.wristVelocity = wristVelocity;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kV = kV;
    this.kG = kG;
    this.kA = kA;
    this.ANGLE_MAX_ACCELERATION = ANGLE_MAX_ACCELERATION;
    this.ANGLE_MAX_VELOCITY = ANGLE_MAX_VELOCITY;
    this.ANGLe_MAX_JERK = ANGLe_MAX_JERK;
    this.motorToWristRotations = motorToWristRotations;
    this.angleOffset = angleOffset;
    this.supplyLimit = supplyLimit;
    this.statorLimit = statorLimit;
    this.WRIST_MIN_DEG = WRIST_MIN_DEG;
    this.WRIST_MAX_DEG = WRIST_MAX_DEG;
  }

  // TODO: Fix later with real values
  public static final WristConstants ShoulderWrist = new WristConstants(
      18,
      19,
      20,
      InvertedValue.CounterClockwise_Positive,
      100000,
      3.669162,
      0.0,
      0.2,
      0.000,
      0.0,
      0.0,
      0.0,
      1000.0,
      1000.0,
      1000.0,
      1.0, // 4.846
      Angle.ofBaseUnits(0.0, Degrees), // AngleOffset
      40,
      80,
      Rotation2d.fromRotations(-0.25), // -90°
      Rotation2d.fromRotations(0.25)); // +90°

  // TODO: Fix later with real values
  public static final WristConstants IntakeWrist = new WristConstants(
      0,
      0,
      0,
      InvertedValue.CounterClockwise_Positive,
      10,
      3.669162,
      0.0,
      0.2,
      0.000,
      0.0,
      0.0,
      0.0,
      1000.0,
      1000.0,
      1000.0,
      1.0,
      Angle.ofBaseUnits(0.0, Degrees), // AngleOffset
      40,
      80,
      Rotation2d.fromRotations(-1.0 / 3.0), // -120°
      Rotation2d.fromRotations(1.0 / 3.0)); // +120°
}
