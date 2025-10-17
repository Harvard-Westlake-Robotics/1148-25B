package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeConstants constants;
  private final String key = "AlgaeIntake";
  private static AlgaeIntake instance;

  SysIdRoutine sysId;
  private Boolean hasAlgae = false;

  public Boolean hasAlgae() {
    return hasAlgae;
  }

  public static AlgaeIntake getInstance() {
    if (instance == null) {
      instance = new AlgaeIntake();
    }
    return instance;
  }

  public AlgaeIntake() {
    this.constants = IntakeConstants.AlgaeIntake;
    io = new IntakeIOTalonFX(constants, 1);
    sysId =
        new SysIdRoutine(
            new Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput(key + "/SysIdState", state.toString())),
            new Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  public IntakeConstants getConstants() {
    return constants;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);
    hasAlgae = inputs.intakeMotorAppliedVolts > 1.0 && inputs.intakeMotorVelocityMPS < 0.2;
  }

  public void runCharacterization(double voltage) {
    io.runCharacterization(voltage);
  }

  public void runVelocity(LinearVelocity velocity) {
    io.runVelocity(velocity);
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
