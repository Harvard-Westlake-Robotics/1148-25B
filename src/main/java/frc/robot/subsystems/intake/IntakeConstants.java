package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeConstants {
  public final int motorId;
  public final InvertedValue motorInverted;
  public final double intakeVelocity;
  public final double outtakeVelocity;
  public double kP;
  public double kI;
  public double kD;
  public double kS;
  public double kV;
  public double kA;
  public double positionkP;
  public double positionkD;
  public int sensor1ID;
  public int sensor2ID;
  public int sensor3ID;
  public int sensor4ID;

  public final double ANGLE_MAX_ACCELERATION;
  public final double ANGLE_MAX_VELOCITY;
  public final double ANGLe_MAX_JERK;
  public final double rotationsToMetersRatio;

  public IntakeConstants(
      int motorId,
      InvertedValue motorInverted,
      double intakeVelocity,
      double outtakeVelocity,
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      int sensor1ID,
      int sensor2ID,
      int sensor3ID,
      int sensor4ID,
      double positionkP,
      double positionkD,
      double ANGLE_MAX_ACCELERATION,
      double ANGLE_MAX_VELOCITY,
      double ANGLe_MAX_JERK,
      double rotationsToMetersRatio) {
    this.motorId = motorId;
    this.motorInverted = motorInverted;
    this.intakeVelocity = intakeVelocity;
    this.outtakeVelocity = outtakeVelocity;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.sensor1ID = sensor1ID;
    this.sensor2ID = sensor2ID;
    this.sensor3ID = sensor3ID;
    this.positionkP = positionkP;
    this.positionkD = positionkD;
    this.ANGLE_MAX_ACCELERATION = ANGLE_MAX_ACCELERATION;
    this.ANGLE_MAX_VELOCITY = ANGLE_MAX_VELOCITY;
    this.ANGLe_MAX_JERK = ANGLe_MAX_JERK;
    this.rotationsToMetersRatio = rotationsToMetersRatio;
  }

  public static final IntakeConstants CoralIntake =
      new IntakeConstants(
          17,
          InvertedValue.Clockwise_Positive,
          8,
          -8,
          8.569162,
          0.0,
          0.0,
          0.3,
          0.0,
          0.0,
          2,
          3,
          4,
          5,
          0.55,
          0.005,
          99999.0,
          99999.0,
          99999.0,
          1.0);

  public static final IntakeConstants AlgaeIntake =
      new IntakeConstants(
          16,
          InvertedValue.CounterClockwise_Positive,
          100,
          -100,
          1.669162,
          0.0,
          0.0,
          0.1761,
          0.12875,
          0.0,
          -1,
          -1,
          -1,
          -1,
          0.0,
          0.0,
          100.0,
          100.0,
          1000000.0,
          1.0 / 16.709);
}
