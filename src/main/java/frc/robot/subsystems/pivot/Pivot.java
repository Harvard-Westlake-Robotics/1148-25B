package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIOTalonFX io1;
  private final PivotIOTalonFX io2;
  private final PivotIOTalonFX io3;
  private final PivotIOInputsAutoLogged inputs1 = new PivotIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged inputs2 = new PivotIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged inputs3 = new PivotIOInputsAutoLogged();

  private final String key = "RealOutputs/Pivot";
  private static Pivot instance;

  SysIdRoutine sysId;

  public static Pivot getInstance() {
    if (instance == null) {
      instance = new Pivot();
    }
    return instance;
  }

  public Pivot() {
    io1 = new PivotIOTalonFX(1, "drive");
    io2 = new PivotIOTalonFX(2, "drive");
    io3 = new PivotIOTalonFX(3, "drive");
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
    Logger.processInputs(key + "/Motor1", inputs1);
    io2.updateInputs(inputs2);
    Logger.processInputs(key + "/Motor2", inputs2);
    io3.updateInputs(inputs3);
    Logger.processInputs(key + "/Motor3", inputs3);
    Logger.recordOutput(key + "/Target Angle", io1.getTargetDegrees());
  }

  public void runCharacterization(double voltage) {
    io1.runCharacterization(voltage);
    io2.runCharacterization(voltage);
    io3.runCharacterization(voltage);
  }

  public void goToAngleClosedLoop(double angle) {
    io1.goToAngleClosedLoop(angle);
    io2.goToAngleClosedLoop(angle);
    io3.goToAngleClosedLoop(angle);
  }

  public void tareAngle(double angle) {
    io1.tareAngle(angle);
    io2.tareAngle(angle);
    io3.tareAngle(angle);
  }

  public double getAngle() {
    return io1.getAnglePivotRots();
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
