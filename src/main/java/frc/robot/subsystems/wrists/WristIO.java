package frc.robot.subsystems.wrists;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean wristMotorConnected = false;
    public double wristPositionRot = 0.0;
    public double wristVelocityRPS = 0.0;
    public double wristAppliedVolts = 0.0;
    public double wristCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  public default void runCharacterization(double voltage) {}

  public default void goToAngleClosedLoop(double angle, double offsetAngle) {}

  public default void tareAngle(double angle) {}
}
