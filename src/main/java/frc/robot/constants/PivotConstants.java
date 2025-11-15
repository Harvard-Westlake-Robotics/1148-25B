package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class PivotConstants {
  // Motor constants
  // ID of the first motor
  public static final int motor1Id = 25;
  public static final int motor2Id = 26;
  public static final int motor3Id = 27;
  public static final String motorCANBusName = "drive";
  public static final InvertedValue motorsInverted = InvertedValue.Clockwise_Positive;
  public static final Current statorLimit = Amps.of(120);
  public static final Current supplyLimit = Amps.of(80);

  // PID constants
  public static double kP = 12;
  public static double kI = 0.0;
  public static double kD = 0.0;
  public static Voltage kS = Volts.of(0.0);
  public static Per<VoltageUnit, AngularVelocityUnit> kV = Volts.per(RotationsPerSecond).ofNative(0.0);
  public static Per<VoltageUnit, AngularAccelerationUnit> kA = Volts.per(RotationsPerSecondPerSecond).ofNative(0.0);
  // Determine kG by gradually increasing phoenix tuner voltage until the Pivot can hold itself up
  // (velocity is 0), then divide the voltage by the cosine of the absolute angle to get kG
  public static Voltage kG = Volts.of(1.5);
  public static GravityTypeValue gravityType = GravityTypeValue.Arm_Cosine;

  // Motion magic constants
  public static final AngularAcceleration motionMagicAcceleration = RotationsPerSecondPerSecond.of(0.25);
  public static final AngularVelocity motionMagicCruiseVelocity = RotationsPerSecond.of(0.3);
  public static final Velocity<AngularAccelerationUnit> motionMagicJerk = RotationsPerSecondPerSecond.per(Second).of(1);

  // Physical constants
  public static final double motorRotationsPerPivotRotationRatio = 112.5;
  public static final Angle angleOffset = Rotations.of(20.43); // AngleOffset
  public static final Angle pivotMinAngle = Rotations.of(0); // Resting on base
  public static final Angle pivotMaxAngle = Rotations.of(0.29);

  // Pivot encoder constants
  public static final int pivotEncoderId = 30;
  public static final String pivotEncoderCANBusName = "drive";
  public static final double pivotEncoderRotationsPerPivotRotationRatio = 1.0;
  public static final Angle pivotEncoderOffset = Rotations.of(0.403);
  public static final SensorDirectionValue pivotEncoderSensorDirection =
      SensorDirectionValue.Clockwise_Positive;
}
