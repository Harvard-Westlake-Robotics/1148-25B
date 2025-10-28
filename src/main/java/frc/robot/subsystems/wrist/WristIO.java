package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean wristMotorConnected = false;
    public double wristPositionDeg = 0.0;
    public double wristVelocityDPS = 0.0;
    public double wristAppliedVolts = 0.0;
    public double wristCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  public default void runVoltage(double voltage) {}

  public default void goToAngleClosedLoop(double angle) {}

  public default void tareAngle(double angle) {}

  public default double getTargetDegrees() {
    return 0.0;
  }
}
