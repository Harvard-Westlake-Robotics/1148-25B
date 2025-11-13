package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.PivotConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIOTalonFX io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private final String key = "Pivot";
  private static Pivot instance;

  SysIdRoutine sysId;

  private final LoggedTunableNumber kP = new LoggedTunableNumber(key + "/kP", PivotConstants.kP);
  private final LoggedTunableNumber kI = new LoggedTunableNumber(key + "/kI", PivotConstants.kI);
  private final LoggedTunableNumber kD = new LoggedTunableNumber(key + "/kD", PivotConstants.kD);
  private final LoggedTunableNumber kS = new LoggedTunableNumber(key + "/kS", PivotConstants.kS);
  private final LoggedTunableNumber kV = new LoggedTunableNumber(key + "/kV", PivotConstants.kV);
  private final LoggedTunableNumber kG = new LoggedTunableNumber(key + "/kG", PivotConstants.kG);
  private final LoggedTunableNumber kA = new LoggedTunableNumber(key + "/kA", PivotConstants.kA);

  private final LoggedTunableNumber motionMagicAcceleration =
      new LoggedTunableNumber(
          key + "/motionMagicAcceleration", PivotConstants.motionMagicAcceleration);
  private final LoggedTunableNumber motionMagicCruiseVelocity =
      new LoggedTunableNumber(
          key + "/motionMagicCruiseVelocity", PivotConstants.motionMagicCruiseVelocity);
  private final LoggedTunableNumber motionMagicJerk =
      new LoggedTunableNumber(key + "/motionMagicJerk", PivotConstants.motionMagicJerk);

  private final LoggedTunableNumber pivotAngleDeg =
      new LoggedTunableNumber("Pivot/pivotAngleDeg", 70);

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

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            io.setTunableConstants(
                kP.get(),
                kI.get(),
                kD.get(),
                kS.get(),
                kV.get(),
                kG.get(),
                kA.get(),
                motionMagicAcceleration.get(),
                motionMagicCruiseVelocity.get(),
                motionMagicJerk.get(),
                Units.degreesToRotations(pivotAngleDeg.get())),
        kP,
        kI,
        kD,
        kS,
        kV,
        kG,
        kA,
        motionMagicAcceleration,
        motionMagicCruiseVelocity,
        motionMagicJerk,
        pivotAngleDeg);
  }

  public void runVoltage(double voltage) {
    // io.runVoltage(voltage);
  }

  public void goToAngleClosedLoop(double angleRots) {
    // io.goToAngleClosedLoop(angleRots);
  }

  public void tareAngle(double angleRots) {
    // io.tareAngle(angleRots);
  }

  @AutoLogOutput
  public double getAngleRots() {
    return Units.degreesToRotations(inputs.pivotPositionDeg);
  }

  public double getAngleDeg() {
    return inputs.pivotPositionDeg;
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
