package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.ElevatorConstants;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io1;
  private final ElevatorIO io2;
  private final ElevatorIOInputsAutoLogged inputs1 = new ElevatorIOInputsAutoLogged();
  private final ElevatorIOInputsAutoLogged inputs2 = new ElevatorIOInputsAutoLogged();

  private DigitalInput dio = new DigitalInput(5);

  private final String key = "Elevator";
  private static Elevator instance = null;
  SysIdRoutine sysId;

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  private Elevator() {
    io1 = new ElevatorIOTalonFX(ElevatorConstants.elevator1Inverted, ElevatorConstants.elevator1ID);
    io2 = new ElevatorIOTalonFX(ElevatorConstants.elevator2Inverted, ElevatorConstants.elevator2ID);
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
    io1.updateInputs(inputs1);
    io2.updateInputs(inputs2);
    Logger.processInputs(key + "/Motor1", inputs1);
    Logger.processInputs(key + "/Motor2", inputs2);

    if (!dio.get() && inputs1.elevatorPositionMeters >= 0.05) {
      io1.tareHeight(0);
      io2.tareHeight(0);
    }
  }

  public void runCharacterization(double voltage) {
    io1.runCharacterization(voltage);
    io2.runCharacterization(voltage);
  }

  public void goToHeightClosedLoop(double height) {
    io1.goToHeightClosedLoop(height);
    io2.goToHeightClosedLoop(height);
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

  public double getCurrentHeight() {
    return inputs1.elevatorPositionMeters;
  }
}
