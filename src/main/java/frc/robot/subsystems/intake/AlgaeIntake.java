package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase {
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeConstants constants;
  private String key;
  private static AlgaeIntake instance;
  private Boolean hasAlgae = false;

  public Boolean getHasAlgae() {
    return hasAlgae;
  }

  public void setHasAlgae(Boolean hasAlgae) {
    this.hasAlgae = hasAlgae;
  }

  public static AlgaeIntake getInstance() {
    if (instance == null) {
      instance = new AlgaeIntake();
    }
    return instance;
  }

  public AlgaeIntake() {
    this.constants = Constants.AlgaeIntake;
    this.key = "RealOutputs/Algae Intake";
    io = new IntakeIOTalonFX(constants);
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
