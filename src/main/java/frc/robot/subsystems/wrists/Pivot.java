package frc.robot.subsystems.wrists;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.WristConstants;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final WristIOTalonFX io1;
  private final WristIOTalonFX io2;
  private final WristIOTalonFX io3;
  private final WristIOInputsAutoLogged inputs1 = new WristIOInputsAutoLogged();
  private final WristIOInputsAutoLogged inputs2 = new WristIOInputsAutoLogged();
  private final WristIOInputsAutoLogged inputs3 = new WristIOInputsAutoLogged();

  private final EncoderIOCANcoder encoderIO;
  private final EncoderIOInputsAutoLogged encoderInputs = new EncoderIOInputsAutoLogged();

  private final WristConstants constants;
  private final String key = "RealOutputs/Pivot";
  private static Pivot instance;

  SysIdRoutine sysId;

  public double getAngle() {
    return encoderIO.getAngle();
  }

  public static Pivot getInstance() {
    if (instance == null) {
      instance = new Pivot();
    }
    return instance;
  }

  public Pivot() {
    this.constants = WristConstants.Pivot;
    io1 = new WristIOTalonFX(constants, 1, "drive");
    io2 = new WristIOTalonFX(constants, 2, "drive");
    io3 = new WristIOTalonFX(constants, 3, "drive");
    encoderIO = new EncoderIOCANcoder();
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
    io1.tareAngle(encoderIO.getAngle());
    io2.tareAngle(encoderIO.getAngle());
    io3.tareAngle(encoderIO.getAngle());
    io1.updateInputs(inputs1);
    Logger.processInputs(key + "/Motor1", inputs1);
    io2.updateInputs(inputs2);
    Logger.processInputs(key + "/Motor2", inputs2);
    io3.updateInputs(inputs3);
    Logger.processInputs(key + "/Motor3", inputs3);
    encoderIO.updateInputs(encoderInputs);
    Logger.processInputs(key + "/Encoder", encoderInputs);
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
