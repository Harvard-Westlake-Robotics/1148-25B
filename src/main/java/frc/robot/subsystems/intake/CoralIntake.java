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

public class CoralIntake extends SubsystemBase {
  private final IntakeIOTalonFX io;
  private final IntakeIOTalonFX io2;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOInputsAutoLogged inputs2 = new IntakeIOInputsAutoLogged();

  private final IntakeConstants constants;
  private final String key = "Subsystems/CoralIntake";
  private static CoralIntake instance;

  SysIdRoutine sysId;
  // Whether we have the coral straight in the intake
  private boolean hasCoralHotdog;
  // Whether we have the coral burger style in the intake
  private boolean hasCoralBurger;

  public Boolean hasCoralHotDog() {
    return hasCoralHotdog;
  }

  public boolean hasCoralBurger() {
    return hasCoralBurger;
  }

  public static CoralIntake getInstance() {
    if (instance == null) {
      instance = new CoralIntake();
    }
    return instance;
  }

  public CoralIntake() {
    this.constants = IntakeConstants.CoralIntake;
    io = new IntakeIOTalonFX(constants, 1);
    io2 = new IntakeIOTalonFX(constants, 2);
    sysId =
        new SysIdRoutine(
            new Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput(key + "/SysIdState", state.toString())),
            new Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io2.updateInputs(inputs2);
    Logger.processInputs(key + "/Motor1", inputs);
    Logger.processInputs(key + "/Motor2", inputs2);
    Logger.recordOutput("Sensor1", getSensor1());
    Logger.recordOutput("Sensor2", getSensor2());
    Logger.recordOutput("Sensor3", getSensor3());
    Logger.recordOutput("Sensor4", getSensor4());
    if (getSensor4() && getSensor2()) {
      hasCoralHotdog = true;
    } else {
      hasCoralHotdog = false;
    }
    if (getSensor1() || getSensor3() || getSensor2()) {
      hasCoralBurger = true;
    } else {
      hasCoralBurger = false;
    }
  }

  public void runCharacterization(double voltage) {
    // io.runVoltage(voltage);
    // io2.runVoltage(-voltage);
  }

  public void runVelocity(LinearVelocity velocity) {
    // io.runVelocity(velocity);
    // io2.runVelocity(velocity.times(-1));
  }

  // Runs motors in the same direction
  public void runVelocityShift(LinearVelocity velocity) {
    // io.runVelocity(velocity);
    // io2.runVelocity(velocity);
  }

  // Shifts the motors by setting them to spin in the same direction
  // Boolean direction -> true is go right, false is go left
  // might not work yet because we're unsure how the shifting will actually need
  // to work
  public void shift(boolean direction, LinearVelocity velocity) {
    // if (direction) {
    //   runVelocityShift(velocity);
    // } else {
    //   runVelocityShift(velocity.times(-1));
    // }
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

  public void yeah() {
    System.out.println("yeah");
  }
}
