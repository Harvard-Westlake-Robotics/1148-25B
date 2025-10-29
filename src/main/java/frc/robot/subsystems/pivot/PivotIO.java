package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public boolean pivotMotorConnected = false;
    public boolean pivotEncoderConnected = false;
    public double pivotPositionDeg = 0.0;
    public double pivotVelocityDPS = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  public default void runVoltage(double voltage) {}

  public default void goToAngleClosedLoop(double pivotAngleRots) {}

  public default void tareAngle(double pivotAngleRots) {}

  public default double getPivotTargetDegrees() {
    return 0.0;
  }
}
