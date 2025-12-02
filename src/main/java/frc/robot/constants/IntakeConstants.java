package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class IntakeConstants {
  // Motor constants
  public final int motorId;
  public final InvertedValue motorInverted;
  public final Current statorLimit;
  public final Current supplyLimit;

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
  public final LinearAcceleration motionMagicAcceleration;
  public final LinearVelocity motionMagicCruiseVelocity;
  public final Velocity<LinearAccelerationUnit> motionMagicJerk;

  // Sensor constants
  public int sensor1ID;
  public int sensor2ID;
  public int sensor3ID;
  public int sensor4ID;

  // Velocity constants
  public final LinearVelocity intakeVelocity;
  public final LinearVelocity outtakeVelocity;
  public final LinearVelocity shiftVelocity;
  // Velocity of the Algae roller when holding coral hamburger
  public final LinearVelocity hamburgerIntakeVelocity;
  public final LinearVelocity hamburgerOuttakeVelocity;

  // Physical constants
  public final double rotationsPerMeterRatio;

  public final Voltage algaeHoldVoltage;

  public IntakeConstants(
      int motorId,
      InvertedValue motorInverted,
      Current statorLimit,
      Current supplyLimit,
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double positionkP,
      double positionkD,
      LinearAcceleration motionMagicAcceleration,
      LinearVelocity motionMagicCruiseVelocity,
      Velocity<LinearAccelerationUnit> motionMagicJerk,
      int sensor1id,
      int sensor2id,
      int sensor3id,
      int sensor4id,
      LinearVelocity intakeVelocity,
      LinearVelocity outtakeVelocity,
      LinearVelocity shiftVelocity,
      LinearVelocity hamburgerIntakeVelocity,
      LinearVelocity hamburgerOuttakeVelocity,
      double rotationsPerMeterRatio,
      Voltage algaeHoldVoltage) {
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
    this.sensor1ID = sensor1id;
    this.sensor2ID = sensor2id;
    this.sensor3ID = sensor3id;
    this.sensor4ID = sensor4id;
    this.intakeVelocity = intakeVelocity;
    this.outtakeVelocity = outtakeVelocity;
    this.shiftVelocity = shiftVelocity;
    this.hamburgerIntakeVelocity = hamburgerIntakeVelocity;
    this.hamburgerOuttakeVelocity = hamburgerOuttakeVelocity;
    this.rotationsPerMeterRatio = rotationsPerMeterRatio;
    this.algaeHoldVoltage = algaeHoldVoltage;
  }

  public static final IntakeConstants CoralIntake =
      new IntakeConstants(
          9,
          InvertedValue.Clockwise_Positive,
          Amps.of(80),
          Amps.of(40),
          0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.5,
          0.0,
          MetersPerSecondPerSecond.of(99999.0),
          MetersPerSecond.of(99999.0),
          MetersPerSecondPerSecond.per(Second).of(99999.0),
          23,
          24,
          22,
          21,
          MetersPerSecond.of(1000),
          MetersPerSecond.of(-1000),
          MetersPerSecond.of(1000),
          MetersPerSecond.of(0),
          MetersPerSecond.of(0),
          9.52 / (4 * Math.PI),
          Volts.of(2));

  public static final IntakeConstants AlgaeIntake =
      new IntakeConstants(
          31,
          InvertedValue.CounterClockwise_Positive,
          Amps.of(120),
          Amps.of(50),
          0.0, // 0.25
          0.0,
          0.08,
          0.0, // 0.1761
          0.0, // 0.12875
          0.0,
          0.0,
          0.0,
          MetersPerSecondPerSecond.of(1000.0),
          MetersPerSecond.of(2000.0),
          MetersPerSecondPerSecond.per(Second).of(1000000.0),
          -1,
          -1,
          -1,
          -1,
          MetersPerSecond.of(2000),
          MetersPerSecond.of(-2000),
          MetersPerSecond.of(0),
          MetersPerSecond.of(-2000),
          MetersPerSecond.of(2000),
          1.0 / 16.709,
          Volts.of(8));
}
