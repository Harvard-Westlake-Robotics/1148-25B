package frc.robot.subsystems.hang;

import com.ctre.phoenix6.signals.InvertedValue;

public class HangConstants {
  public static final int motorId = 15;
  public static final InvertedValue motorInverted = InvertedValue.Clockwise_Positive;
  public static final double kP = 0.5;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.1;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double hangVelocity = 6;
  public static final double rotationsToMetersRatio = (1);
}
