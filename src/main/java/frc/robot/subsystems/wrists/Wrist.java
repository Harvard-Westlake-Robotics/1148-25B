package frc.robot.subsystems.wrists;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.WristConstants;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIOTalonFX io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final WristConstants constants;
  private final String key = "RealOutputs/Wrist";
  private static Wrist instance;

  SysIdRoutine sysId;

  public static Wrist getInstance() {
    if (instance == null) {
      instance = new Wrist();
    }
    return instance;
  }

  public Wrist() {
    this.constants = WristConstants.Wrist;
    io = new WristIOTalonFX(constants, 1, "rio");
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
  }

  public void runCharacterization(double voltage) {
    io.runCharacterization(voltage);
  }

  public void goToAngleClosedLoop(double angle) {
    io.goToAngleClosedLoop(angle);
  }

  public void tareAngle(double angle) {
    io.tareAngle(angle);
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
