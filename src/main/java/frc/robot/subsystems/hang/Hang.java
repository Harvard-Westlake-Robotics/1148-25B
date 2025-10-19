package frc.robot.subsystems.hang;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import org.littletonrobotics.junction.Logger;

public class Hang extends SubsystemBase {
  private final HangIO io;
  private final HangIOInputsAutoLogged inputs = new HangIOInputsAutoLogged();

  private final String key = "RealOutputs/Hang";
  private static Hang instance = null;
  SysIdRoutine sysId;

  private boolean hasBar = false;
  // TODO: Make private and encapsulate
  public Servo servo = new Servo(7);

  public Boolean hasBar() {
    return hasBar;
  }

  public void setHasBar(Boolean hasBar) {
    this.hasBar = hasBar;
  }

  public static Hang getInstance() {
    if (instance == null) {
      instance = new Hang();
    }
    return instance;
  }

  public Hang() {
    io = new HangIOTalonFX();
    sysId =
        new SysIdRoutine(
            new Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput(key + "/SysIdState", state.toString())),
            new Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);
    hasBar = inputs.motorAppliedVolts > 1.0 && inputs.motorVelocityMPS < 0.2;
  }

  public void runCharacterization(double voltage) {
    io.runCharacterization(voltage);
  }

  public void runVelocityClosedLoop(LinearVelocity velocity) {
    io.runVelocityClosedLoop(velocity);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }
}
