package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeConstants {
  // Motor constants
  public final int motorId;
  public final InvertedValue motorInverted;
  public final double intakeVelocity;
  public final double outtakeVelocity;
  public final double shiftVelocity;
  public final int statorLimit;
  public final int supplyLimit;

  // PID constants
  public double kP;
  public double kI;
  public double kD;
  public double kS;
  public double kV;
  public double kA;
  public double positionkP;
  public double positionkD;

  // Motion magic constants
  public final double motionMagicAcceleration;
  public final double motionMagicCruiseVelocity;
  public final double motionMagicJerk;

  // Sensor constants
  public int sensor1ID;
  public int sensor2ID;
  public int sensor3ID;
  public int sensor4ID;

  // Physical constants
  public final double rotationsToMetersRatio;

  public IntakeConstants(int motorId, InvertedValue motorInverted, double intakeVelocity, double outtakeVelocity,
      double shiftVelocity, int statorLimit, int supplyLimit, double kP, double kI, double kD, double kS, double kV,
      double kA, double positionkP, double positionkD, double motionMagicAcceleration, double motionMagicCruiseVelocity,
      double motionMagicJerk, int sensor1id, int sensor2id, int sensor3id, int sensor4id,
      double rotationsToMetersRatio) {
    this.motorId = motorId;
    this.motorInverted = motorInverted;
    this.intakeVelocity = intakeVelocity;
    this.outtakeVelocity = outtakeVelocity;
    this.shiftVelocity = shiftVelocity;
    this.statorLimit = statorLimit;
    this.supplyLimit = supplyLimit;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.positionkP = positionkP;
    this.positionkD = positionkD;
    this.motionMagicAcceleration = motionMagicAcceleration;
    this.motionMagicCruiseVelocity = motionMagicCruiseVelocity;
    this.motionMagicJerk = motionMagicJerk;
    sensor1ID = sensor1id;
    sensor2ID = sensor2id;
    sensor3ID = sensor3id;
    sensor4ID = sensor4id;
    this.rotationsToMetersRatio = rotationsToMetersRatio;
  }

  public static final IntakeConstants CoralIntake = new IntakeConstants(
      17,
      InvertedValue.Clockwise_Positive,
      100,
      -100,
      100,
      120,
      50,
      8.569162,
      0.0,
      0.0,
      0.3,
      0.0,
      0.0,
      0.55,
      0.005,
      99999.0,
      99999.0,
      99999.0,
      2,
      3,
      4,
      5,
      1.0);

  public static final IntakeConstants AlgaeIntake = new IntakeConstants(
      16,
      InvertedValue.CounterClockwise_Positive,
      100,
      -100,
      0,
      120,
      50,
      1.669162,
      0.0,
      0.0,
      0.1761,
      0.12875,
      0.0,
      0.0,
      0.0,
      100.0,
      100.0,
      1000000.0,
      -1,
      -1,
      -1,
      -1,
      1.0 / 16.709);
}
