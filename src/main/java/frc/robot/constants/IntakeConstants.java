package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.LinearVelocity;

public class IntakeConstants {
  // Motor constants
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

  // Velocity constants
  public final LinearVelocity intakeVelocity;
  public final LinearVelocity outtakeVelocity;
  public final LinearVelocity shiftVelocity;
  // Velocity of the top roller when intaking
  public final LinearVelocity hamburgerIntakeVelocity;

  // Physical constants
  public final double rotationsPerMeterRatio;

  public IntakeConstants(
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
      double positionkP,
      double positionkD,
      double motionMagicAcceleration,
      double motionMagicCruiseVelocity,
      double motionMagicJerk,
      int sensor1id,
      int sensor2id,
      int sensor3id,
      int sensor4id,
      LinearVelocity intakeVelocity,
      LinearVelocity outtakeVelocity,
      LinearVelocity shiftVelocity,
      LinearVelocity hamburgerIntakeVelocity,
      double rotationsPerMeterRatio) {
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
    this.positionkP = positionkP;
    this.positionkD = positionkD;
    this.motionMagicAcceleration = motionMagicAcceleration;
    this.motionMagicCruiseVelocity = motionMagicCruiseVelocity;
    this.motionMagicJerk = motionMagicJerk;
    sensor1ID = sensor1id;
    sensor2ID = sensor2id;
    sensor3ID = sensor3id;
    sensor4ID = sensor4id;
    this.intakeVelocity = intakeVelocity;
    this.outtakeVelocity = outtakeVelocity;
    this.shiftVelocity = shiftVelocity;
    this.hamburgerIntakeVelocity = hamburgerIntakeVelocity;
    this.rotationsPerMeterRatio = rotationsPerMeterRatio;
  }

  // TODO: ID These
  public static final IntakeConstants CoralIntake =
      new IntakeConstants(
          9,
          InvertedValue.Clockwise_Positive,
          80,
          40,
          10,
          0.0,
          0.0,
          0.1,
          0.0,
          0.0,
          0.5,
          0.0,
          99999.0,
          99999.0,
          99999.0,
          2,
          3,
          4,
          5,
          MetersPerSecond.of(20),
          MetersPerSecond.of(-20),
          MetersPerSecond.of(20),
          MetersPerSecond.of(20),
          9.52 / (4 * Math.PI));

  public static final IntakeConstants AlgaeIntake =
      new IntakeConstants(
          31,
          InvertedValue.CounterClockwise_Positive,
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
          MetersPerSecond.of(20),
          MetersPerSecond.of(-20),
          MetersPerSecond.of(0),
          MetersPerSecond.of(0),
          7.11 / (2 * Math.PI));
}
