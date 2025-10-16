package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;
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
  private IntakeIOTalonFX io;
  private IntakeIOTalonFX io2;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOInputsAutoLogged inputs2 = new IntakeIOInputsAutoLogged();
  private IntakeConstants constants;
  private static CoralIntake instance;
  // Whether we have the coral straight in the intake
  private boolean hasCoralHotdog;
  // Whether we have the coral burger style in the intake
  private boolean hasCoralBurger;
  SysIdRoutine sysId;

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
    Logger.processInputs("Coral Intake/Motor1", inputs);
    Logger.processInputs("Coral Intake/Motor2", inputs2);
    if (getSensor4() && getSensor2()) {
      hasCoralHotdog = true;
    } else {
      hasCoralHotdog = false;
    }
    if (getSensor1() && getSensor2() && getSensor3()) {
      hasCoralBurger = true;
    } else {
      hasCoralBurger = false;
    }
  }

  public void setVelocity(LinearVelocity velocity) {
    io.runVelocity(velocity);
    io2.runVelocity(velocity.times(-1));
  }

  // Sets motors to the same direction
  public void setVelocityShift(LinearVelocity velocity) {
    io.runVelocity(velocity);
    io2.runVelocity(velocity);
  }

  public void setVelocityMPS(double velocity) {
    LinearVelocity v = LinearVelocity.ofBaseUnits(velocity, MetersPerSecond);
    setVelocity(v);
  }

  public void setVelocityShiftMPS(double velocity) {
    LinearVelocity v = LinearVelocity.ofBaseUnits(velocity, MetersPerSecond);
    setVelocityShift(v);
  }

  public void runVoltage(double volts) {
    io.runCharacterization(volts);
    io2.runCharacterization(-volts);
  }

  public void setVoltageShift(double volts) {
    io.runCharacterization(volts);
    io2.runCharacterization(volts);
  }

  // Shifts the motors by setting them to spin in the same direction
  // Boolean direction -> true is go right, false is go left
  // might not work yet because we're unsure how the shifting will actually need
  // to work
  public void shift(boolean direction, double velocity) {
    if (direction) {
      setVelocityShiftMPS(velocity);
    } else {
      setVelocityShiftMPS(velocity * -1);
    }
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

  public Boolean hasCoralHotDog() {
    return hasCoralHotdog;
  }

  public boolean hasCoralBurger() {
    return hasCoralBurger;
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
