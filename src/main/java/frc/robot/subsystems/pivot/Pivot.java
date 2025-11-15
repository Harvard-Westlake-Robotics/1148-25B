package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
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
  private final LoggedTunableNumber kS = new LoggedTunableNumber(key + "/kS", PivotConstants.kS.in(Volts));
  private final LoggedTunableNumber kV = new LoggedTunableNumber(key + "/kV", PivotConstants.kV.in(Volts.per(RotationsPerSecond)));
  private final LoggedTunableNumber kG = new LoggedTunableNumber(key + "/kG", PivotConstants.kG.in(Volts));
  private final LoggedTunableNumber kA = new LoggedTunableNumber(key + "/kA", PivotConstants.kA.in(Volts.per(RotationsPerSecondPerSecond)));

  private final LoggedTunableNumber motionMagicAcceleration =
      new LoggedTunableNumber(
          key + "/motionMagicAcceleration", PivotConstants.motionMagicAcceleration.in(RotationsPerSecondPerSecond));
  private final LoggedTunableNumber motionMagicCruiseVelocity =
      new LoggedTunableNumber(
          key + "/motionMagicCruiseVelocity", PivotConstants.motionMagicCruiseVelocity.in(RotationsPerSecond));
  private final LoggedTunableNumber motionMagicJerk =
      new LoggedTunableNumber(key + "/motionMagicJerk", PivotConstants.motionMagicJerk.in(RotationsPerSecondPerSecond.per(Second)));

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
            new Mechanism((voltage) -> runVoltage(voltage), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);
    Logger.recordOutput(key + "/TargetAngle", io.getPivotTarget());

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            io.setTunableConstants(
                kP.get(),
                kI.get(),
                kD.get(),
                Volts.of(kS.get()),
                Volts.per(RotationsPerSecond).ofNative(kV.get()),
                Volts.of(kG.get()),
                Volts.per(RotationsPerSecondPerSecond).ofNative(kA.get()),
                RotationsPerSecondPerSecond.of(motionMagicAcceleration.get()),
                RotationsPerSecond.of(motionMagicCruiseVelocity.get()),
                RotationsPerSecondPerSecond.per(Second).of(motionMagicJerk.get()),
                Degrees.of(pivotAngleDeg.get())),
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

  public void runVoltage(Voltage voltage) {
    // io.runVoltage(voltage);
  }

  public void goToAngleClosedLoop(Angle pivotAngle) {
    // io.goToAngleClosedLoop(pivotAngle);
  }

  public void tareAngle(Angle pivotAngle) {
    // io.tareAngle(pivotAngle);
  }

  @AutoLogOutput
  public Angle getAngle() {
    return inputs.pivotAngle;
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runVoltage(Volts.of(0.0))).withTimeout(1.0).andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runVoltage(Volts.of(0.0))).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }
}
