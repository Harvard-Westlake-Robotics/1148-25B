package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  private IntakeIOTalonFX io;
  private IntakeIOTalonFX io2;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOInputsAutoLogged inputs2 = new IntakeIOInputsAutoLogged();
  private IntakeConstants constants;
  private String key;
  private static CoralIntake instance;
  private boolean hasCoral;
  SysIdRoutine sysId;

  public static CoralIntake getInstance() {
    if (instance == null) {
      instance = new CoralIntake();
    }
    return instance;
  }

  public CoralIntake() {
    this.constants = IntakeConstants.CoralIntake;
    this.key = "Coral Intake";
    io = new IntakeIOTalonFX(constants, 1);
    io2 = new IntakeIOTalonFX(constants, 2);
    sysId =
        new SysIdRoutine(
            new Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput(key + "/SysIdState", state.toString())),
            new Mechanism((voltage) -> runVoltage(voltage.in(Volts)), null, this));
  }

  public IntakeConstants getConstants() {
    return constants;
  }

  public void periodic() {
    Logger.recordOutput("Sensor 1", getSensor1());
    Logger.recordOutput("Sensor 2", getSensor2());
    Logger.recordOutput("Sensor 3", getSensor3());
    Logger.recordOutput("Sensor 4", getSensor4());
    io.updateInputs(inputs);
    io2.updateInputs(inputs2);
    Logger.processInputs(key + "/Motor1", inputs);
    Logger.processInputs(key + "/Motor2", inputs2);
    if (getSensor4() && getSensor2()) {
      hasCoral = true;
    } else {
      hasCoral = false;
    }
  }

  public void setVelocity(LinearVelocity velocity) {
    io.runVelocity(velocity);
    io2.runVelocity(velocity.times(-1));
  }

  public void setVelocityShift(LinearVelocity velocity) {
    io.runVelocity(velocity);
    io2.runVelocity(velocity);
  }

  public void runVoltage(double volts) {
    io.runCharacterization(volts);
    io2.runCharacterization(-volts);
  }

  public void setVoltageShift(double volts) {
    io.runCharacterization(volts);
    io2.runCharacterization(volts);
  }

  public Boolean getSensor1() {
    return io.getSensor1();
  }

  public Boolean getSensor2() {
    return io.getSensor2();
  }

  public Boolean getSensor3() {
    return io.getSensor3();
  }

  public Boolean getSensor4() {
    return io.getSensor4();
  }

  public Boolean hasCoral() {
    return hasCoral;
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
