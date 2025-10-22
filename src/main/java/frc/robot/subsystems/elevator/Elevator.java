package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final String key = "Elevator";
  private static Elevator instance = null;
  private DigitalInput dio = new DigitalInput(5);
  SysIdRoutine sysId;

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  private Elevator() {
    io = new ElevatorIOTalonFX();
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

    // TODO: What does this do?
    if (!dio.get() && inputs.elevator1PositionMeters >= 0.05) {
      io.tareHeight(0);
    }
  }

  public void runCharacterization(double voltage) {
    io.runCharacterization(voltage);
  }

  public void goToHeightClosedLoop(double height) {
    io.goToHeightClosedLoop(height);
  }

  public double getExtension() {
    return inputs.elevator1PositionMeters;
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

  public double getTarget() {
    return io.getTarget();
  }

  public double getCurrentHeight() {
    return inputs.elevator1PositionMeters;
  }
}
