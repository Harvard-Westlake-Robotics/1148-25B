package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIOTalonFX io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private final String key = "Pivot";
  private static Pivot instance;

  SysIdRoutine sysId;

  public static Pivot getInstance() {
    if (instance == null) {
      instance = new Pivot();
    }
    return instance;
  }

  public Pivot() {
    io = new PivotIOTalonFX();
    sysId =
        new SysIdRoutine(
            new Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput(key + "/SysIdState", state.toString())),
            new Mechanism((voltage) -> runVoltage(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);
    Logger.recordOutput(key + "/TargetAngle", io.getPivotTargetDegrees());
  }

  public void runVoltage(double voltage) {
    io.runVoltage(voltage);
  }

  public void goToAngleClosedLoop(double angleRots) {
    io.goToAngleClosedLoop(angleRots);
  }

  public void tareAngle(double angleRots) {
    io.tareAngle(angleRots);
  }

  @AutoLogOutput
  public double getAngleRots() {
    return Units.degreesToRotations(inputs.pivotPositionDeg);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runVoltage(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }
}
