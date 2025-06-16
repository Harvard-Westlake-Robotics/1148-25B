package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeMotorConnected = false;
    public double intakeMotorPositionMeters = 0.0;
    public double intakeMotorVelocityMPS = 0.0;
    public double intakeMotorAppliedVolts = 0.0;
    public double intakeMotorCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runVelocity(AngularVelocity velocity) {}

  public default void runVelocity(LinearVelocity velocity) {}

  public default void runCharacterization(double volts) {}

  public Boolean getSensor1();

  public Boolean getSensor2();

  public Boolean getSensor3();

  public Boolean getSensor4();
}
