package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.IntakeConstants;

import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase {
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeConstants constants;
  private String key;
  private static AlgaeIntake instance;
  private Boolean hasAlgae = false;
  SysIdRoutine sysId;

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
    this.constants = Constants.AlgaeIntake;
    this.key = "Algae Intake";
    io = new IntakeIOTalonFX(constants, 1);
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
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);
    hasAlgae = inputs.intakeMotorAppliedVolts > 1.0 && inputs.intakeMotorVelocityMPS < 0.2;
  }

  public void setVelocity(LinearVelocity velocity) {
    io.runVelocity(velocity);
  }

  public void runVoltage(double volts) {
    io.runCharacterization(volts);
  }
}
