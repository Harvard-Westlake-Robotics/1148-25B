package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.InvertedValue;

public class ElevatorConstants {
    public static final int elevator1ID = 13;
    public static final int elevator2ID = 14;
    public static final InvertedValue elevator2Inverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue elevator1Inverted = InvertedValue.Clockwise_Positive;
    public static double kP = 2.0;
    public static double kI = 0.01;
    public static double kD = 0.00;
    public static double kS = 0.1;
    public static double kV = 0.0;
    public static double kG = 0.0;
    public static double kA = 0.0;
    public static final double elevatorForwardSoftLimitRotations = 55;
    public static final double elevatorReverseSoftLimitRotations = 0.0;
    public static final double rotationsToMetersRatio = (1);
    public static final double elevatorGroundOffsetMeters = 0.2125;
  }
