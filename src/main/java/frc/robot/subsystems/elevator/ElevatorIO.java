package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean elevatorConnected = false;
    public double elevatorPositionMeters = 0.0;
    public double elevatorVelocityMPS = 0.0;
    public double elevatorAppliedVolts = 0.0;
    public double elevatorCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void runVoltage(double voltage) {}

  public default void goToHeightClosedLoop(double height) {}

  /** Sets the motor's internally stored position to what it is fed */
  public default void tareHeight(double height) {}

  public default double getTarget() {
    return 0.0;
  }
}
